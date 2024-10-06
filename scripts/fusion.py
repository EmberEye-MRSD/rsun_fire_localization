#!/usr/bin/env python3

import rospy
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
import tf.transformations as tft

# Refined Camera Matrix and Distortion Coefficients
refined_camera_matrix = np.array([[408.87350878, 0, 309.32990709],
                                  [0, 408.94608921, 240.2088536],
                                  [0, 0, 1]])

refined_dist_coeffs = np.array([[-4.02814957e-01],
                                [ 2.15711407e-01],
                                [-9.41079706e-05],
                                [-2.24618698e-04],
                                [-6.48024124e-02]])

# refined_dist_coeffs = np.array([[0.37050391], [0.12533725], [0.00107224], [-0.00104793], [0.00000000]])
# refined_camera_matrix = np.array([[413.20436653, 0, 311.92044067],
#             [0, 413.21024107, 238.53571275],
#             [0, 0, 1]])

# Variables for translation (XYZ) and rotation (RPY) between the frames
translation = np.array([0.048,-0.039,-0.020])  # Example translation (in meters)
rotation_rpy = 0 * np.array([-3.50, -3.50, 0.0]) * np.pi / 180  # Convert degrees to radians

# Convert RPY to a rotation matrix
rotation_matrix = tft.euler_matrix(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])[:3, :3]

# Path to ROS bag file
bag_file = '/home/gangadhar-nageswar/embereye_ws/src/rsun_fire_localization/scripts/2024-09-29-13-42-30.bag'  # Update with the correct path
image_topic = '/thermal_left/image'
pointcloud_topic = '/camera/depth/color/points'

bridge = CvBridge()

# Publisher for filtered point cloud
pub = None

def transform_point_cloud(points, rotation_matrix, translation):
    """ Vectorized point cloud transformation. """
    # Apply rotation and translation in a single operation
    transform_point_cloud =np.dot(points[:, :3], rotation_matrix.T) + translation

    # publish this point cloud to a new topic
    header = Header(stamp=rospy.Time.now(), frame_id="camera_depth_optical_frame")
    transformed_cloud = pcl2.create_cloud_xyz32(header, transform_point_cloud)
    transformed_pc_pub.publish(transformed_cloud)

    return transform_point_cloud

def untransform_point_cloud(points, rotation_matrix, translation):
    """ Vectorized reverse point cloud transformation. """
    return np.dot(points[:, :3] - translation, rotation_matrix)

def project_points_to_image(transformed_points, camera_matrix, dist_coeffs):
    """ Vectorized projection of 3D points onto 2D image plane using OpenCV """
    # Reshape points to (N, 1, 3) for cv2.projectPoints
    points_reshaped = transformed_points[:, np.newaxis, :]
    projected_points, _ = cv2.projectPoints(points_reshaped, np.zeros((3, 1)), np.zeros((3, 1)), camera_matrix, dist_coeffs)
    return projected_points.squeeze()

def filter_points_by_mask(projected_points_2d, points_3d, mask):
    """ Vectorized filtering of 3D points based on 2D projected positions. """
    valid_indices = (
        (projected_points_2d[:, 0] >= 0) &
        (projected_points_2d[:, 0] < mask.shape[1]) &
        (projected_points_2d[:, 1] >= 0) &
        (projected_points_2d[:, 1] < mask.shape[0]) &
        (mask[projected_points_2d[:, 1].astype(int), projected_points_2d[:, 0].astype(int)] > 0)
    )
    return points_3d[valid_indices]

def process_bag():
    global pub

    # Open the ROS bag
    bag = rosbag.Bag(bag_file)

    for topic, msg, t in bag.read_messages(topics=[image_topic, pointcloud_topic]):
        if topic == image_topic:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, "passthrough").copy()
                cv_image[cv_image < 23000] = 0
                cv_image[cv_image > 23000] = 255
                # cv_image = np.clip(cv_image, 0, 255)  # Clip intensity values for valid range
                mask = cv_image.astype(np.uint8)  # Generate binary mask
            except CvBridgeError as e:
                rospy.logerr(f"Error converting image: {e}")
                continue

        elif topic == pointcloud_topic:
            # Read point cloud data and convert to NumPy array
            point_cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(point_cloud))

            # Transform the point cloud from the lidar frame to the camera frame
            transformed_points = transform_point_cloud(points, rotation_matrix, translation)

            # Project the transformed points onto the image plane
            projected_points_2d = project_points_to_image(transformed_points, refined_camera_matrix, refined_dist_coeffs)

            # Filter points based on the mask
            filtered_points = filter_points_by_mask(projected_points_2d, transformed_points, mask)

            # If filtered points are found, compute centroid and print it
            if filtered_points.size > 0:
                centroid = np.mean(filtered_points, axis=0)
                rospy.loginfo(f"Centroid of filtered points: {centroid}")

            # Untransform the filtered points back to the original frame
            untransformed_points = untransform_point_cloud(filtered_points, rotation_matrix, translation)

            # Publish the filtered and untransformed points as a PointCloud2 message
            header = Header(stamp=rospy.Time.now(), frame_id=msg.header.frame_id)
            filtered_cloud = pcl2.create_cloud_xyz32(header, untransformed_points)
            pub.publish(filtered_cloud)

    bag.close()

if __name__ == '__main__':
    rospy.init_node('filtered_pointcloud_node', anonymous=True)
    pub = rospy.Publisher('/filtered_pointcloud', PointCloud2, queue_size=1)
    transformed_pc_pub = rospy.Publisher('/transformed_pointcloud', PointCloud2, queue_size=1)

    try:
        process_bag()
    except rospy.ROSInterruptException:
        pass
