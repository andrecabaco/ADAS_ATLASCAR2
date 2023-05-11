#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from math import cos, sin, radians


def lidar1_callback(msg, args):
    lidar1_frame_id, lidar1_data, pub_point_cloud = args
    lidar1_data = msg
    merge_lidar_data_and_publish(lidar1_data, lidar2_data, lidar1_frame_id, pub_point_cloud)


def lidar2_callback(msg, args):
    lidar2_frame_id, lidar2_data, pub_point_cloud = args
    lidar2_data = msg
    merge_lidar_data_and_publish(lidar1_data, lidar2_data, lidar2_frame_id, pub_point_cloud)


def merge_lidar_data_and_publish(lidar1_data, lidar2_data, frame_id, pub_point_cloud):
    if lidar1_data is None or lidar2_data is None:
        return

    # Convert lidar data to PointCloud2 messages
    point_cloud1 = convert_to_point_cloud(lidar1_data, frame_id)
    point_cloud2 = convert_to_point_cloud(lidar2_data, frame_id)

    # Merge the two PointCloud2 messages
    merged_point_cloud = merge_point_clouds(point_cloud1, point_cloud2)

    # Publish the merged PointCloud2 message
    pub_point_cloud.publish(merged_point_cloud)


def convert_to_point_cloud(lidar_data, frame_id):
    # Convert lidar data to PointCloud2 message
    point_cloud = pc2.create_cloud_xyz32(header=rospy.Header(frame_id=frame_id),
                                         points=[[reading * cos(radians(angle)),
                                                   reading * sin(radians(angle)),
                                                   0.0] for angle, reading in enumerate(lidar_data.ranges)])

    return point_cloud


def merge_point_clouds(point_cloud1, point_cloud2):
    # Merge two PointCloud2 messages into one
    merged_point_cloud = pc2.read_points(point_cloud1) + pc2.read_points(point_cloud2)

    # Convert the merged points to a new PointCloud2 message
    new_point_cloud = pc2.create_cloud_xyz32(header=rospy.Header(frame_id="merged_lidar"),
                                             points=list(merged_point_cloud))

    return new_point_cloud


if __name__ == "__main__":
    rospy.init_node("lidar_merger")

    lidar1_frame_id = rospy.get_param("~lidar1_frame_id", "lidar1")
    lidar2_frame_id = rospy.get_param("~lidar2_frame_id", "lidar2")
    lidar1_data = None
    lidar2_data = None

    pub_point_cloud = rospy.Publisher("~merged_point_cloud", PointCloud2, queue_size=10)

    rospy.Subscriber("~lidar1_scan", LaserScan, lidar1_callback, (lidar1_frame_id, lidar1_data, pub_point_cloud))
    rospy.Subscriber("~lidar2_scan", LaserScan, lidar2_callback, (lidar2_frame_id, lidar2_data, pub_point_cloud))

    rospy.spin()


