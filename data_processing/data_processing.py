#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import numpy as np
import sys
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_matrix

def check_clusters_in_region(clusters):
    region_x_min = 2.0
    region_x_max = 13.5  # 10m + 7m/2 (from base_link)
    region_y_min = -1.0
    region_y_max = 1.0  # 10m / 2
    region_z_min = 0.3
    region_z_max = 0.8  
    for i, cluster_points in enumerate(clusters):
        centroid = np.mean(cluster_points, axis=0)
        print (centroid)
        if region_x_min <= centroid[0] <= region_x_max and region_y_min <= centroid[1] <= region_y_max and region_z_min <= centroid[2] <= region_z_max:
            return True
    return False

def publish_markers(clusters, cluster_labels, reference_frame, prev_distances, prev_times):
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
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

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

        # Calculate the distance between the cluster centroid and the furthest point in the cluster
        centroid = np.mean(cluster_points, axis=0)
        distance = np.linalg.norm(centroid - reference_frame)
        furthest_distance_x = np.max(np.abs(cluster_points[:, 0] - centroid[0]))
        furthest_distance_y = np.max(np.abs(cluster_points[:, 1] - centroid[1]))

        # Add Text Markers with cluster number, distance, and number of points
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"  # Update with your desired frame ID
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "text_markers"
        text_marker.id = i
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = centroid[0]
        text_marker.pose.position.y = centroid[1]
        text_marker.pose.position.z = centroid[2] + 0.2  # Adjust the height of the text above the cluster centroid
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        # Calculate the relative velocity
        if i in prev_distances and i in prev_times:
            prev_distance = prev_distances[i]
            prev_time = prev_times[i]
            current_time = rospy.Time.now()
            delta_t = (current_time - prev_time).to_sec()
            relative_velocity = (prev_distance - distance) / delta_t
            text_marker.text = f"Cluster {i + 1}\nDistance: {distance:.2f} m\nPoints: {len(cluster_points)}\nVelocity: {relative_velocity:.2f} m/s"
        else:
            # No previous distance or time recorded, use the current distance as the initial value
            prev_distances[i] = distance
            prev_times[i] = rospy.Time.now()
            text_marker.text = f"Cluster {i + 1}\nDistance: {distance:.2f} m\nPoints: {len(cluster_points)}"

        print("Cluster " + str(i + 1) + " - Distance: " + str(distance) + " m | Velocity: " + str(relative_velocity) + " m/s")

        # Update the previous distance and time for the next iteration
        prev_distances[i] = distance
        prev_times[i] = rospy.Time.now()

        # Add an ellipse marker
        ellipse_marker = Marker()
        ellipse_marker.header.frame_id = "base_link"  # Update with your desired frame ID
        ellipse_marker.header.stamp = rospy.Time.now()
        ellipse_marker.ns = "ellipse_markers"
        ellipse_marker.id = i
        ellipse_marker.type = Marker.CYLINDER
        ellipse_marker.action = Marker.ADD
        ellipse_marker.pose.position.x = centroid[0]
        ellipse_marker.pose.position.y = centroid[1]
        ellipse_marker.pose.position.z = centroid[2]
        ellipse_marker.pose.orientation.w = 1.0
        ellipse_marker.scale.x = 2 * furthest_distance_x  # Twice the furthest distance in x direction as diameter
        ellipse_marker.scale.y = 2 * furthest_distance_y  # Twice the furthest distance in y direction as diameter
        ellipse_marker.scale.z = 0.05  # The height of the cylinder
        ellipse_marker.color.r = marker.color.r
        ellipse_marker.color.g = marker.color.g
        ellipse_marker.color.b = marker.color.b
        ellipse_marker.color.a = 0.3  # Slightly transparent

        marker_array.markers.append(marker)
        marker_array.markers.append(text_marker)
        marker_array.markers.append(ellipse_marker)

    # Check if there is any cluster found in the specified region
    found_in_region = check_clusters_in_region(clusters)
    print("Cluster found in region:", found_in_region)

    # Add a region marker
    region_marker = Marker()
    region_marker.header.frame_id = "base_link"  # Update with your desired frame ID
    region_marker.header.stamp = rospy.Time.now()
    region_marker.ns = "region_markers"
    region_marker.id = 0
    region_marker.type = Marker.CUBE
    region_marker.action = Marker.ADD
    region_marker.pose.position.x = 7.0  # 1m away from the base_link frame along the x-axis
    region_marker.pose.position.y = 0.0
    region_marker.pose.position.z = 0.3
    region_marker.pose.orientation.w = 1.0
    region_marker.scale.x = 10.0  # 2m in the x direction
    region_marker.scale.y = 2.0  # 10m in the y direction
    region_marker.scale.z = 0.5  # The height of the cube
    if found_in_region:
        region_marker.color.r = 1.0
        region_marker.color.g = 0.0
        region_marker.color.b = 0.0
    else:
        region_marker.color.r = 0.0
        region_marker.color.g = 1.0
        region_marker.color.b = 0.0
    region_marker.color.a = 0.3  # Slightly transparent

    marker_array.markers.append(region_marker)

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
        if len(cluster_points) >= 30:
            print("Probability of another vehicle or a large object")
        else:
            print("Probability of a pedestrian or a vulnerable user of the road")

    # Get the transformation matrix of the reference frame
    tf_matrix = quaternion_matrix([0, 0, 0, 1])  # Replace with the actual quaternion

    # Extract the translation component from the transformation matrix
    reference_frame = tf_matrix[:3, 3]

    # Initialize dictionaries to store the previous distances and times for each cluster
    global prev_distances, prev_times
    if 'prev_distances' not in globals():
        prev_distances = {}
    if 'prev_times' not in globals():
        prev_times = {}

    publish_markers(clusters, labels, reference_frame, prev_distances, prev_times)

def main():
    rospy.init_node('pointcloud_cluster', anonymous=True)
    rospy.Subscriber('/merged_cloud', PointCloud2, pointcloud_callback)
    global marker_pub
    marker_pub = rospy.Publisher('/cluster_markers', MarkerArray, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
