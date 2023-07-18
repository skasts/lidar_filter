#!/usr/bin/env python3

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

class LidarFilterNode:

    def __init__(self):
        rospy.init_node('lidar_filter_node')

        # Get box dimensions from parameters
        x_min = rospy.get_param("~x_min", -1.0)
        x_max = rospy.get_param("~x_max", 1.0)
        y_min = rospy.get_param("~y_min", -1.0)
        y_max = rospy.get_param("~y_max", 1.0)
        z_min = rospy.get_param("~z_min", -0.0)
        z_max = rospy.get_param("~z_max", 1.0)

        # Get topic names from parameters
        input_topic = rospy.get_param("~input_topic", "/rslidar_points_unfiltered")
        output_topic = rospy.get_param("~output_topic", "/rslidar_points")

        corner_points = [   (x_min, y_min, z_min),
                            (x_min, y_min, z_max),
                            (x_min, y_max, z_min),
                            (x_min, y_max, z_max),
                            (x_max, y_min, z_min),
                            (x_max, y_min, z_max),
                            (x_max, y_max, z_min),
                            (x_max, y_max, z_max)]

        # Create a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.transformed_corner_points = []
        for corner_point in corner_points:
            # Create a PoseStamped message with the corner point coordinates in the base_link frame
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = corner_point[0]
            pose.pose.position.y = corner_point[1]
            pose.pose.position.z = corner_point[2]
            pose.pose.orientation.w = 1.0

            # Use the transform_pose() function to transform the pose to the rslidar frame
            transformed_pose = self.tf_buffer.transform(pose, 'rslidar', rospy.Duration(2.0))
            
            self.tf_buffer.transform(pose, 'rslidar')
            # Extract the transformed corner point coordinates
            transformed_corner = (
                transformed_pose.pose.position.x,
                transformed_pose.pose.position.y,
                transformed_pose.pose.position.z
            )
            # Append the transformed corner point to the list
            self.transformed_corner_points.append(transformed_corner)

        # Set up the subscribers and publishers
        self.subscriber = rospy.Subscriber(input_topic, PointCloud2, self.point_cloud_callback)
        self.publisher = rospy.Publisher(output_topic, PointCloud2, queue_size=10)


    def point_cloud_callback(self, msg):

        # Filter the points based on the transformed box dimensions in rslidar frame
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        filtered_points = []
        for [x, y, z] in points:
            if (not(
                self.transformed_corner_points[0][0] <= x <= self.transformed_corner_points[7][0] and
                self.transformed_corner_points[0][1] <= y <= self.transformed_corner_points[7][1] and
                self.transformed_corner_points[0][2] <= z <= self.transformed_corner_points[7][2]
            )):
                filtered_points.append((x, y, z))

        # # Create a new point cloud message with the filtered points in rslidar frame
        # filtered_cloud_msg = pc2.create_cloud_xyz32(msg.header, self.transformed_corner_points + filtered_points)
        filtered_cloud_msg = pc2.create_cloud_xyz32(msg.header, filtered_points)

        # Publish the filtered point cloud
        self.publisher.publish(filtered_cloud_msg)

if __name__ == '__main__':
    try:
        node = LidarFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
