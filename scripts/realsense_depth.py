#!/usr/bin/env python3

import rospy
import pyrealsense2 as rs
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import open3d as o3d

import ipdb

st = ipdb.set_trace

'''
Pipeline:
1. Read the points from realsense
2. Apply tranformation to the points from the camera_depth_optical_frame to the camera_color_frame
3. Project all the points from the camera_color_frame to the image
4. Publish the image
'''

class RealSenseNode:
    def __init__(self):
        rospy.init_node('realsense_reader_node', anonymous=True)

        # Bridge between ROS and OpenCV
        self.bridge = CvBridge()

        self.point_cloud = None
        self.image = None

        # subscribers
        # self.point_cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.point_cloud_callback)
        self.point_cloud_sub = rospy.Subscriber('/thermal/fire/points', PointCloud2, self.point_cloud_callback)
        self.image_sub = rospy.Subscriber('/thermal_left/image', Image, self.image_callback)

        # publisher for publishing the transformed point cloud
        # self.point_cloud_pub = rospy.Publisher('/camera/depth/color/points_transformed', PointCloud2, queue_size=10)

        # transform
        # self.tf_buffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # camera intrinsics
        # self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # publish the projected image
        self.projected_img_pub = rospy.Publisher('/camera/color/projected_image', Image, queue_size=10)

        # publish segmented image
        self.projected_img_pub = rospy.Publisher('/camera/color/segmented_image', Image, queue_size=10)

        # publish the point cloud
        self.segmented_point_cloud_pub = rospy.Publisher('/segmented_point_cloud', PointCloud2, queue_size=10)

        # rosparams for color segmentation
        rospy.set_param('/color_min', [0, 50, 50])
        rospy.set_param('/color_max', [10, 255, 255])

        # self.D = np.array([0.37050391, 0.12533725, 0.00107224, -0.00104793, -0.00000000])
        # self.K = np.array([[413.20436653, 0, 311.92044067],
        #            [0, 413.21024107, 238.53571275],
        #            [0, 0, 1]])

        # OpenCV refined camera parameters
    
    def camera_info_callback(self, data):
        self.camera_parameters = data
        
        self.K = np.array(data.K).reshape(3, 3)
        self.D = np.array(data.D)
    
    def point_cloud_callback(self, data):
        # st()
        self.point_cloud = data

        # # transform all the points from the camera_depth_optical_frame to the camera_color_optical_frame
        # try:
        #     transform = self.tf_buffer.lookup_transform('camera_color_optical_frame', 'camera_depth_optical_frame', rospy.Time(0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     return
        
        # self.transformed_cloud = do_transform_cloud(self.point_cloud, transform)
        
        # # publish the transformed point cloud
        # self.point_cloud_pub.publish(self.transformed_cloud)

        transformed_points = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"), skip_nans=True)
        transformed_points = np.array(list(transformed_points))
        # transformed_points = self.downsample_voxel_grid(transformed_points, 0.05)

        # print(f"max value: {np.max(transformed_points, axis=0)}")
        # print(f"min value: {np.min(transformed_points, axis=0)}")

        # project the points to the image
        # self.K = np.array([[408.87350878, 0., 309.32990709],
        #           [0., 408.94608921, 240.2088536],
        #           [0., 0., 1.]])
        
        # self.D = np.array([[-4.02814957e-01], [2.15711407e-01], [-9.41079706e-05], [-2.24618698e-04], [-6.48024124e-02]])

        self.K = np.array([[408.87350878, 0, 309.32990709],
                            [0, 408.94608921, 240.2088536],
                            [0, 0, 1]])

        self.D = np.array([[-4.02814957e-01],
                            [ 2.15711407e-01],
                            [-9.41079706e-05],
                            [-2.24618698e-04],
                            [-6.48024124e-02]])
        
        projected_img_points, _ = cv2.projectPoints(objectPoints=transformed_points, rvec=np.zeros((3, 1)), tvec=np.zeros((3, 1)), cameraMatrix=self.K, distCoeffs=self.D)

        # convert the projected points to integer values
        projected_img_points = projected_img_points.astype(int)

        # # obatin the segmented image and the points
        color_min = rospy.get_param('/color_min')
        color_max = rospy.get_param('/color_max')

        masked_image = self.segment_image(self.image, color_min, color_max)
        # cv2.imshow('masked_image', masked_image)
        # cv2.waitKey(1)

        # obtain the points from the masked image as a numpy array of shape (n, 2)
        masked_points = np.argwhere(masked_image == 255)

        # st()

        # search for the points in the projected_img_points and return the corresponding 3D points
        pc_indices = self.search_points(projected_img_points, masked_points)

        # st()

        # get the points from the projected_img_points corresponding to the pc_indices
        pc_points = transformed_points[pc_indices]

        # get the mean of the pc_points and print it
        pc_mean = np.mean(pc_points, axis=0)
        print(pc_mean)

        # publish these points as a point cloud
        self.publish_point_cloud(pc_points)
    
    def publish_point_cloud(self, points):
        # create a ROS pointcloud2 message
        header = self.point_cloud.header
        header.frame_id = 'thermal_left_optical_frame'
        pc2_msg = pc2.create_cloud_xyz32(header, points)

        # publish the point cloud
        self.segmented_point_cloud_pub.publish(pc2_msg)
    
    def search_points(self, projected_img_points, masked_points):
        # search for the points in the projected_img_points and return the corresponding 3D points
        # pc_indices = []
        # for point in masked_points:
        #     for i, proj_point in enumerate(projected_img_points):
        #         # print(F"Point: {point}, Proj Point: {proj_point[0]}")
        #         if np.array_equal(proj_point[0], point):
        #             pc_indices.append(i)
        #             print(f"Found point: {point}")
        #             break
        
        # return pc_indices
        projected_img_points = projected_img_points.reshape(-1, 2)
        # print(f"projected points shape: {projected_img_points.shape}")
        # print(f"masked points shape: {masked_points.shape}")

        match_mask = np.all(masked_points[:, None] == projected_img_points, axis=-1)
        indices = np.where(match_mask)[1]

        return indices

        # matches = np.any(np.all(projected_img_points[:, np.newaxis] == masked_points, axis=2), axis=1)
        # matching_indices = np.where(matches)[0]

        # return matching_indices
    
    def downsample_voxel_grid(self, array, voxel_size):
        """Downsamples a 3D NumPy array using a voxel grid filter."""
        # Convert the array to an Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(array)

        # Create the voxel grid and downsample
        downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)

        # Convert back to a NumPy array
        downsampled_array = np.asarray(downpcd.points)

        return downsampled_array
    
    def segment_image(self, image, color_min, color_max):
        # # convert to LAB color space
        # lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

        # # Perform Otsu threshold on the A-channel 
        # th = cv2.threshold(lab[:,:,1], 127, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        # # publish the thresholded image
        # self.segmented_img_pub.publish(self.bridge.cv2_to_imgmsg(th, 'mono8'))

        # # exchange the vertices of the mask, i.e. row to coloumn and vice versa
        # mask = np.transpose(th)

        min_val = 23000
        max_val = 40000

        # print(f"min val: {image.min()} | max val: {image.max()}")

        cv_img = self.image.copy()

        cv_img[cv_img < min_val] = 0
        cv_img[cv_img > min_val] = 255
        # cv_img[]
        # cv_img[cv_img > max_val] = 0

        # # cv_img -= min_val
        # # cv_img = cv_img / (max_val - min_val)
        # # cv_img = cv_img * 255

        # # img = Image.fromarray(cv_img)
        # # cv_img = np.array(img.convert('RGB'))

        cv_img = cv_img.astype(np.uint8)
        # cv2.imshow('image', cv_img)
        # cv2.waitKey(0)

        return cv_img

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
        self.segment_image(self.image, [0, 50, 50], [10, 255, 255])

if __name__ == '__main__':
    try:
        realsense_node = RealSenseNode()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()