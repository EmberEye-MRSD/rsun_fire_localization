#include <vector>
#include <iostream>
#include <random>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types_conversion.h>


class RealSenseFireLocalization
{
private:
    ros::NodeHandle nh;

    ros::Publisher fire_pointcloud_pub;

    ros::Subscriber realsense_pointcloud_sub;
    ros::Subscriber thermal_image_sub;

    sensor_msgs::PointCloud2 transformed_points;

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 415.68148228, 0, 312.98599158,
                                                       0, 415.74859039, 254.63140201,
                                                       0, 0, 1);

    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -4.16123804e-01, 2.33440434e-01, -1.19852069e-04, 5.89824552e-04, -7.49109237e-02);

    cv::Mat mask;

    // Multi hotspot localization
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_points;
    std::vector<cv::Mat> mask_clusters;
    ros::Publisher hotspots_pose_array_pub;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfListener;

    double hotspot_threshold;


    // temp variables
    cv::Mat mask_clusters_image;
    ros::Publisher mask_clusters_pub;

public:
    RealSenseFireLocalization() : tfListener(tf_buffer)
    {
        // Initialize ROS publishers and subscribers
        fire_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/fire_point_cloud", 1);
        hotspots_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/rsun/hotspot/poses", 1);

        realsense_pointcloud_sub = nh.subscribe("/camera/depth/color/downsampled_points", 1, &RealSenseFireLocalization::pointCloudCallback, this);
        thermal_image_sub = nh.subscribe("/flir_boson/image_raw", 1, &RealSenseFireLocalization::thermalImageCallback, this);

        // Get the initial value of the threshold parameter from the parameter server
        if (!nh.getParam("/hotspot_thres", hotspot_threshold)) {
            ROS_WARN("Parameter /hotspot_thres not set, using default value of 24500");
            nh.setParam("/hotspot_thres", 24500);
            hotspot_threshold = 24500;
        }

        // temp pub for mask clusters
        mask_clusters_pub = nh.advertise<sensor_msgs::Image>("/mask_clusters", 1);
    }

    void thermalImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // Update the threshold value from the parameter server before applying threshold
            nh.getParam("/hotspot_thres", hotspot_threshold);

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");

            cv::threshold(cv_ptr->image, mask, hotspot_threshold, 255, cv::THRESH_BINARY);
            mask.convertTo(mask, CV_8U);

            findClusters(mask, mask_clusters);

            // print the number of clusters
            ROS_INFO("Number of clusters: %zu", mask_clusters.size());

            // cnvert the mask image to ros image msg
            cv_bridge::CvImage mask_clusters_image_msg;
            mask_clusters_image_msg.header.stamp = ros::Time::now();
            mask_clusters_image_msg.header.frame_id = "flir_boson_optical_frame";
            mask_clusters_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
            mask_clusters_image_msg.image = mask.clone();
            mask_clusters_pub.publish(mask_clusters_image_msg.toImageMsg());


        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void filterPointsByMask(const std::vector<cv::Point2d>& projected_points, const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& filtered_points) {
        // Clear the previous points and resize the vector to match the number of clusters
        filtered_points.clear();
        filtered_points.resize(mask_clusters.size());

        // Initialize each filtered point cloud pointer
        for (size_t i = 0; i < filtered_points.size(); ++i) {
            filtered_points[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        }

        // Iterate over all the projected points and assign them to the appropriate cluster
        for (size_t i = 0; i < projected_points.size(); ++i) {
            int x = static_cast<int>(projected_points[i].x);
            int y = static_cast<int>(projected_points[i].y);
            for (size_t j = 0; j < mask_clusters.size(); ++j) {
                if (x >= 0 && x < mask_clusters[j].cols && y >= 0 && y < mask_clusters[j].rows && mask_clusters[j].at<uchar>(y, x) > 0) {
                    filtered_points[j]->points.push_back(points->points[i]);
                    break; // Break to avoid assigning the same point to multiple clusters
                }
            }
        }
        
    }


    void findClusters(const cv::Mat& binary_image, std::vector<cv::Mat>& clusters) {
        clusters.clear();
        // Label the clusters using connected components
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(binary_image, labels, stats, centroids);

        // Iterate over each label (ignoring the background, label 0)
        for (int label = 1; label < num_labels; ++label) {
            // Extract the mask for the current label
            cv::Mat cluster_mask = (labels == label);
            
            // Remove small clusters
            int cluster_size = stats.at<int>(label, cv::CC_STAT_AREA);
            if (cluster_size < 100) {
                continue; // Skip this cluster if it's too small
            }

            clusters.push_back(cluster_mask.clone());
        }
    }

    void projectPointsToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, std::vector<cv::Point2d>& projected_points) {
        std::vector<cv::Point3d> points_3d;
        for (const auto& point : points->points) {
            points_3d.push_back(cv::Point3d(point.x, point.y, point.z));
        }
        cv::projectPoints(points_3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix, dist_coeffs, projected_points);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        if (mask.empty()) return;

        // Transform the pointcloud from camera_depth_optical_frame to thermal_optical_frame
        geometry_msgs::TransformStamped thermal_realsense_transform;
        try {
            thermal_realsense_transform = tf_buffer.lookupTransform("flir_boson_optical_frame", msg->header.frame_id, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Transform point cloud
        pcl_ros::transformPointCloud("flir_boson_optical_frame", thermal_realsense_transform.transform, *msg, transformed_points);

        std::vector<cv::Point2d> projected_points;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points_pcl(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_points, *transformed_points_pcl);

        projectPointsToImage(transformed_points_pcl, camera_matrix, dist_coeffs, projected_points);

        filterPointsByMask(projected_points, transformed_points_pcl, filtered_points);

        if (!filtered_points.empty()) {
            for (size_t i = 0; i < filtered_points.size(); ++i) {
                if (!filtered_points[i]->points.empty()) {
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*filtered_points[i], centroid);
                    // ROS_INFO("Centroid of filtered points (%zu): (%f, %f, %f)", i, centroid[0], centroid[1], centroid[2]);
                }
            }
        }

        if (!filtered_points.empty()) {
            geometry_msgs::PoseArray pose_array_msg;
            pose_array_msg.header.stamp = ros::Time::now();
            pose_array_msg.header.frame_id = "flir_boson_optical_frame";

            for (size_t i = 0; i < filtered_points.size(); ++i) {
                if (!filtered_points[i]->points.empty()) {
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*filtered_points[i], centroid);

                    // Create a Pose for the centroid
                    geometry_msgs::Pose pose;
                    pose.position.x = centroid[0];
                    pose.position.y = centroid[1];
                    pose.position.z = centroid[2];

                    // Orientation as (0, 0, 0) (roll, pitch, yaw)
                    pose.orientation.x = 0.0;
                    pose.orientation.y = 0.0;
                    pose.orientation.z = 0.0;
                    pose.orientation.w = 1.0;  // Identity quaternion

                    // Add the pose to the PoseArray message
                    pose_array_msg.poses.push_back(pose);
                }
            }

            // Publish the PoseArray message
            hotspots_pose_array_pub.publish(pose_array_msg);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_hotspot_localization");
    RealSenseFireLocalization realsense_fire_localization;
    ros::spin();
    return 0;
}