#include <actionlib/client/simple_action_client.h>
#include <final_assignment_msgs/enshu4_armAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class BehaviorManager
{
public:
  BehaviorManager()
    : arm_action_client_("arm"),
    move_base_action_client_("move_base"),
    tf_listener_(tf_buffer_),
    state_(State::EXPLORING),
    is_object_detected_(false)
  {
    ros::NodeHandle nh;

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    object_position_sub_ = nh.subscribe(
      "object_position", 1, &BehaviorManager::objectPositionCB, this);

    ros::NodeHandle pnh("~");

    global_frame_ = pnh.param<std::string>("global_frame", "odom");
    robot_base_frame_ = pnh.param<std::string>("robot_base_frame", "base_link");
    approach_distance_ = pnh.param<double>("approach_distance", 0.2);

    if (!move_base_action_client_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("move_base action server timeout");
    }

    ROS_INFO("Exploring");
  }

  void run()
  {
    switch (state_)
    {
    case State::EXPLORING:
      if (is_object_detected_)
      {
        state_ = State::TARGETING;
        ROS_INFO("Targeting");
      }
      else
      {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.angular.z = 0.2;
        cmd_vel_pub_.publish(cmd_vel);
      }
      break;

    case State::TARGETING:
      if (std::abs(object_position_wrt_base_.y()) < 0.2)
      {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = global_frame_;
        goal.target_pose.header.stamp = ros::Time::now();
        tf2::toMsg(global_to_approach_goal_tf_, goal.target_pose.pose);
        move_base_action_client_.sendGoal(goal);
        state_ = State::APPROACHING;
        ROS_INFO("Approaching");
      }
      else
      {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.angular.z = std::copysign(0.1, object_position_wrt_base_.y());
        cmd_vel_pub_.publish(cmd_vel);
      }
      break;

    case State::APPROACHING:
      if (move_base_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        final_assignment_msgs::enshu4_armGoal goal;
        tf2::toMsg(object_position_wrt_base_ * 1000, goal.target);
        goal.grasp = 1;
        arm_action_client_.sendGoal(goal);
        state_ = State::GRASPING;
        ROS_INFO("Grasping");
      }
      else if (move_base_action_client_.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        is_object_detected_ = false;
        state_ = State::EXPLORING;
      }
      break;

    case State::GRASPING:
      if (arm_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = global_frame_;
        goal.target_pose.header.stamp = ros::Time::now();
        tf2::toMsg(DROP_POSE, goal.target_pose.pose);
        move_base_action_client_.sendGoal(goal);
        state_ = State::CARRYING;
        ROS_INFO("Carrying");
      }
      else if (arm_action_client_.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        is_object_detected_ = false;
        state_ = State::EXPLORING;
      }
      break;

    case State::CARRYING:
      if (move_base_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
        move_base_action_client_.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        final_assignment_msgs::enshu4_armGoal goal;
        tf2::toMsg(DROP_ARM_TARGET * 1000, goal.target);
        goal.grasp = 0;
        arm_action_client_.sendGoal(goal);
        state_ = State::DROPPING;
        ROS_INFO("Dropping");
      }
      break;

    case State::DROPPING:
      if (arm_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
        arm_action_client_.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        is_object_detected_ = false;
        state_ = State::EXPLORING;
        ROS_INFO("Exploring");
      }
      break;

    default:
      break;
    }
  }

private:
  enum class State { EXPLORING, TARGETING, APPROACHING, GRASPING, CARRYING, DROPPING };

  static const tf2::Transform DROP_POSE;
  static const tf2::Vector3 DROP_ARM_TARGET;

  void objectPositionCB(const geometry_msgs::PointStamped& position)
  {
    tf2::Stamped<tf2::Transform> global_to_base_tf, base_to_camera_tf;
    try
    {
      tf2::fromMsg(tf_buffer_.lookupTransform(
        global_frame_, robot_base_frame_, ros::Time(0)), global_to_base_tf);
      tf2::fromMsg(tf_buffer_.lookupTransform(
        robot_base_frame_, position.header.frame_id, ros::Time(0)), base_to_camera_tf);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }

    tf2::Vector3 object_position_wrt_camera;
    tf2::fromMsg(position.point, object_position_wrt_camera);
    object_position_wrt_base_ = base_to_camera_tf * object_position_wrt_camera;
    ROS_INFO_STREAM(object_position_wrt_base_.x() << ", " << object_position_wrt_base_.y() << ", " << object_position_wrt_base_.z());

    tf2::Transform base_to_approach_goal_tf(
      { {0, 0, 1}, std::atan2(object_position_wrt_base_.y(), object_position_wrt_base_.x()) },
      object_position_wrt_base_.normalized() *
      (object_position_wrt_base_.length() - approach_distance_));

    global_to_approach_goal_tf_ = global_to_base_tf * base_to_approach_goal_tf;

    is_object_detected_ = true;
  }

  ros::Publisher cmd_vel_pub_;

  ros::Subscriber object_position_sub_;

  actionlib::SimpleActionClient<final_assignment_msgs::enshu4_armAction>
    arm_action_client_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    move_base_action_client_;

  tf2_ros::Buffer tf_buffer_;

  tf2_ros::TransformListener tf_listener_;

  State state_;

  bool is_object_detected_;

  std::string global_frame_;

  std::string robot_base_frame_;

  double approach_distance_;

  tf2::Vector3 object_position_wrt_base_;

  tf2::Transform global_to_approach_goal_tf_;
};

const tf2::Transform BehaviorManager::DROP_POSE({ {0, 0, 1}, M_PI }, { 0, 0, 0 });
const tf2::Vector3 BehaviorManager::DROP_ARM_TARGET(0.5, 0, 0.2);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "behavior_manager");

  BehaviorManager behavior_manager;

  ros::Rate rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    behavior_manager.run();

    rate.sleep();
  }

  return 0;
}