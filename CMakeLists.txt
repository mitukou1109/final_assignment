cmake_minimum_required(VERSION 3.0.2)
project(final_assignment)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    final_assignment_msgs
    geometry_msgs
    move_base_msgs
    roscpp
    tf2
    tf2_ros
)

catkin_package(
  DEPENDS
    actionlib
    final_assignment_msgs
    geometry_msgs
    move_base_msgs
    roscpp
    tf2
    tf2_ros
)

add_executable(behavior_manager
  src/behavior_manager.cpp
)
target_include_directories(behavior_manager
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(behavior_manager
  ${catkin_LIBRARIES}
)