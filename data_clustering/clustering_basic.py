#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
import math

from collections import namedtuple

# Define a named tuple for the point
Point3D = namedtuple('Point3D', ['x', 'y', 'z'])

# ...

def msgReceivedCallback(msg, publisher):
    rospy.loginfo("I received a new point cloud message")

    threshold_distance = 0.5
    clusters = []
    cluster_idxs = [0]
    clusters.append(cluster_idxs)

    points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    points = [Point3D(*p) for p in points]  # Convert to namedtuples

    # Clustering
    for idx in range(1, len(points)):
        x, y, z = points[idx]

        prev_x, prev_y, prev_z = points[idx - 1]

        distance = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2 + (z - prev_z) ** 2)
        if distance > threshold_distance:
            clusters.append([idx])
        else:
            clusters[-1].append(idx)

    # Rest of the code remains the same


    # Create a Marker for visualization
    marker = Marker()
    marker.header = msg.header
    marker.ns = 'clusters'
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.MODIFY

    marker.pose.orientation.w = 1
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01

    # Generate a unique color for each cluster
    cluster_colors = []
    for _ in range(len(clusters)):
        color = ColorRGBA()
        color.r = random.random()
        color.g = random.random()
        color.b = random.random()
        color.a = 0.3
        cluster_colors.append(color)

    # Calculate the minimum distance to the reference frame for each cluster
    min_distances = []
    for cluster in clusters:
        cluster_points = [points[idx] for idx in cluster]
        min_distance = min(math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2) for p in cluster_points)
        min_distances.append(min_distance)

    # Add markers for clusters and their corresponding minimum distances
    for cluster_idx, (cluster, min_distance) in enumerate(zip(clusters, min_distances)):
        color = cluster_colors[cluster_idx]
        marker.id = cluster_idx
        marker.colors.extend([color] * len(cluster))
        for idx in cluster:
            p = Point()
            p.x, p.y, p.z = points[idx]
            marker.points.append(p)

        # Add text marker for distance
        text_marker = Marker()
        text_marker.header = msg.header
        text_marker.ns = 'distance_labels'
        text_marker.id = cluster_idx
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.MODIFY
        text_marker.pose.position = p
        text_marker.pose.position.z += 0.2  # Adjust the height of the text
        text_marker.pose.orientation.w = 1
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f'{min_distance:.2f}m'

        publisher.publish(text_marker)

    publisher.publish(marker)


def main():
    rospy.init_node('clustering', anonymous=False)
    publisher = rospy.Publisher('~clusters', Marker, queue_size=1)
    subscriber = rospy.Subscriber('/merged_cloud', PointCloud2, lambda msg: msgReceivedCallback(msg, publisher))
    rospy.spin()

if __name__ == '__main__':
    main()