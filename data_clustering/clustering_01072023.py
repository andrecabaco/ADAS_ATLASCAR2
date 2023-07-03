#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import numpy as np
import sys
from geometry_msgs.msg import Point

def publish_markers(clusters, cluster_labels):
    marker_array = MarkerArray()

    for i, cluster_points in enumerate(clusters):
        # Create a new marker for each cluster
        marker = Marker()
        marker.header.frame_id = "base_link"  # Update with your desired frame ID
        marker.header.stamp = rospy.Time.now()
        marker.ns = "clusters"
        marker.id = i
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # Set the color based on the cluster label
        if cluster_labels[i] == -1:
            marker.color.r = 1.0  # Noise points will be red
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            # Generate a random color for each cluster
            np.random.seed(i)
            marker.color.r = np.random.uniform(0, 1)
            marker.color.g = np.random.uniform(0, 1)
            marker.color.b = np.random.uniform(0, 1)
        marker.color.a = 1.0  # Solid color

        # Set the points of the marker to the cluster points
        marker.points = [Point(p[0], p[1], p[2]) for p in cluster_points]

        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)

def pointcloud_callback(data):
    # Convert PointCloud2 message to numpy array
    point_cloud = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(point_cloud))

    # Perform DBSCAN clustering
    clustering = DBSCAN(eps=0.3, min_samples=10).fit(points)
    labels = clustering.labels_
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    print("Number of clusters:", n_clusters)

    # Process the clusters
    clusters = []
    for i in range(n_clusters):
        cluster_points = points[labels == i]
        clusters.append(cluster_points)
        print("Cluster", i + 1, "points:", len(cluster_points))
        if len(cluster_points) >= 20:
            print("Probability of another vehicle or a large object")
        else:
            print("Probability of a pedestrian or a vulnerable user of the road")

    publish_markers(clusters, labels)

def main():
    rospy.init_node('pointcloud_cluster', anonymous=True)
    rospy.Subscriber('/merged_cloud', PointCloud2, pointcloud_callback)
    global marker_pub
    marker_pub = rospy.Publisher('/cluster_markers', MarkerArray, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()

