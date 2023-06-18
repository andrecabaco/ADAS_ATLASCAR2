#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import numpy as np
import sys

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
    for i in range(n_clusters):
        cluster_points = points[labels == i]
        min_distance = sys.float_info.max  # Set initial minimum distance to a large value

        # Calculate minimum distance within the cluster
        for p1 in cluster_points:
            for p2 in cluster_points:
                if not np.array_equal(p1, p2):  # Compare values instead of identity
                    distance = np.linalg.norm(p1 - p2)
                    if distance < min_distance:
                        min_distance = distance

        #print("Cluster", i + 1, "points:", cluster_points)
        print("Minimum distance within Cluster", i + 1, ":", min_distance)

def main():
    rospy.init_node('pointcloud_cluster', anonymous=True)
    rospy.Subscriber('/merged_cloud', PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == '__main__':
    main()


