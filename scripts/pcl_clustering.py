#!/usr/bin/env python

# Import modules
from pcl_helper import *
from pcl_filter import *

# TODO: Define functions as required

def split_cloud(cloud):
    
    #Downsample the cloud to reduce computation cost
    downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.01)

    #Focus on regoin of interest
    filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud, name_axis = 'z', min_axis = 0.6, max_axis = 1.1)

    table_cloud, objects_cloud = do_ransac_plane_segmentation(filtered_cloud, max_distance = 0.01)

    return objects_cloud, table_cloud

def get_clusters(cloud, tolerance, min_size, max_size):

    tree = cloud.make_kdtree()

    extraction_object = cloud.make_EuclideanClusterExtraction()

    extraction_object.set_ClusterTolerance(tolerance)
    extraction_object.set_MinClusterSize(min_size)
    extraction_object.set_MaxClusterSize(max_size)
    extraction_object.set_SearchMethod(tree)

    clusters = extraction_object.Extract()
    return clusters

def get_colored_clusters(clusters, cloud):
  
    # Get a random unique colors for each object
    number_of_clusters = len(clusters)
    colors = get_color_list(number_of_clusters)

    colored_points = []

    # Assign a color for each point
    # Points with the same color belong to the same cluster
    for cluster_id, cluster in enumerate(clusters):
        for c, i in enumerate(cluster):
            x, y, z = cloud[i][0], cloud[i][1], cloud[i][2]
            color = rgb_to_float(colors[cluster_id])
            colored_points.append([x, y, z, color])
    
    return colored_points

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg) 

    objects_cloud, table_cloud = split_cloud(cloud) 

    colorless_cloud = XYZRGB_to_XYZ(objects_cloud)

    clusters = get_clusters(colorless_cloud, tolerance = 0.05, min_size = 100, max_size = 1500)

    colored_points = get_colored_clusters(clusters, colorless_cloud)

    # Create a cloud with each cluster of points having the same color
    clusters_cloud = pcl.PointCloud_PointXYZRGB()
    clusters_cloud.from_list(colored_points)

    # TODO: Convert PCL data to ROS messages
    objects_msg = pcl_to_ros(objects_cloud)
    table_msg = pcl_to_ros(table_cloud)
    clusters_msg = pcl_to_ros(clusters_cloud)

    # TODO: Publish ROS messages
    objects_publisher.publish(objects_msg)
    table_publisher.publish(table_msg)
    clusters_publisher.publish(clusters_msg)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    subscriber = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size = 1)
    
    # TODO: Create Publishers
    objects_publisher = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
    table_publisher = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
    clusters_publisher = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
