import pcl

def do_voxel_grid_filter(point_cloud, LEAF_SIZE = 0.01):
    '''
    Returns Downsampled version of a point cloud
    The bigger the leaf size the less information retained
    '''
    voxel_filter = point_cloud.make_voxel_grid_filter()
    voxel_filter.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    return voxel_filter.filter()

def do_passthrough_filter(point_cloud, name_axis = 'z', min_axis = 0.6, max_axis = 1.1):
    '''
    Returns only the point cloud information at a specific range of a specific axis
    '''
    pass_filter = point_cloud.make_passthrough_filter()
    pass_filter.set_filter_field_name(name_axis)
    pass_filter.set_filter_limits(min_axis, max_axis)
    return pass_filter.filter()



def do_ransac_plane_segmentation(point_cloud, max_distance = 0.01):
    '''
    Use RANSAC planse segmentation to separate plane and not plane points
    Returns inliers (plane) and outliers (not plane)
    '''
    segmenter = point_cloud.make_segmenter()
    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_distance_threshold(max_distance)

    inlier_indices, coefficients = segmenter.segment()

    inliers = point_cloud.extract(inlier_indices, negative = False)
    outliers = point_cloud.extract(inlier_indices, negative = True)

    return inliers, outliers
