#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped


class ObjectCoord:
    def __init__(self):
        self.pub_coord = rospy.Publisher("/object_position", PointStamped, queue_size=1)
        self.sub_point_clouds = rospy.Subscriber(
            "/object_detection/points", PointCloud2, self.update_points
        )
        self.detect_range = 1.0  # [m]

    def update_points(self, msg):
        points = pc2.read_points(msg, skip_nans=True)
        points = np.asarray(list(points))
        # print(points.shape)

        if points.shape[0] != 0:
            q25, q75 = np.percentile(points, [25, 75], axis=0)
            iqr = q75 - q25

            lower = q25[2] - (iqr[2] * 1.5)
            upper = q75[2] + (iqr[2] * 1.5)

            masked_points = points[(points[:,2] <= upper) * (points[:,2] >= lower)]

            # object_coord = np.mean(points, axis=0)
            object_coord = np.mean(masked_points, axis=0)
            if np.all(object_coord == 0.0) or object_coord[2] >= self.detect_range:
                return
            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "camera_depth_optical_frame"
            msg.point.x = object_coord[0]
            msg.point.y = object_coord[1]
            msg.point.z = object_coord[2]
            self.pub_coord.publish(msg)


def main():
    rospy.loginfo("Start to calculate coord of detected object.")
    rospy.init_node("object_coord")
    ObjectCoord()
    rospy.spin()


if __name__ == "__main__":
    main()
