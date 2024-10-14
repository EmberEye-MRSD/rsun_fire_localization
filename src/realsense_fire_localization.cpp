#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <vector>
#include <pcl_ros/transforms.h>

#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types_conversion.h>

#include <random>

class RealSenseFireLocalization
{
private:
    ros::NodeHandle nh;

    ros::Publisher fire_pointcloud_pub;
    ros::Publisher flir_pointcloud_pub;

    ros::Subscriber depth_sub;
    ros::Subscriber thermal_image_sub;

    ros::Timer processing_timer;

    cv_bridge::CvImage* cv_bridge;

    sensor_msgs::CameraInfo camera_info;
    sensor_msgs::PointCloud2 latest_pointcloud;
    sensor_msgs::PointCloud2 transformed_points;
    sensor_msgs::PointCloud2 fire_points;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfListener;
    

    // old camera matrix
    // cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 408.87350878, 0, 309.32990709,
    //                                                    0, 408.94608921, 240.2088536,
    //                                                    0, 0, 1);



    // cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.402814957, 0.215711407, -0.0000941079706, -0.000224618698, -0.0648024124);

    //new camera matrix
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 415.68148228, 0, 312.98599158,
                                                   0, 415.74859039, 254.63140201,
                                                   0, 0, 1);

    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -4.16123804e-01, 2.33440434e-01, -1.19852069e-04, 5.89824552e-04, -7.49109237e-02);


    cv::Mat mask;
    bool pointcloud_received;

public:
    RealSenseFireLocalization() : tfListener(tf_buffer), pointcloud_received(false) {
        // Initialize ROS
        nh = ros::NodeHandle();

        // Initialize publishers
        fire_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/fire_point_cloud", 1);
        flir_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/flir/points", 1);

        // Initialize subscribers
        depth_sub = nh.subscribe("/camera/depth/color/points", 1, &RealSenseFireLocalization::pointCloudCallback, this);
        thermal_image_sub = nh.subscribe("/flir_boson/image_raw", 1, &RealSenseFireLocalization::thermalImageCallback, this);

        // Timer for periodic processing
        processing_timer = nh.createTimer(ros::Duration(0.033), &RealSenseFireLocalization::timerCallback, this);
    }

    void thermalImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
            cv::threshold(cv_ptr->image, mask, 23000, 255, cv::THRESH_BINARY);
            mask.convertTo(mask, CV_8U);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        latest_pointcloud = *msg;
        pointcloud_received = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!pointcloud_received || mask.empty()) return;

        // Transform the pointcloud from camera_depth_optical_frame to thermal_optical_frame
        geometry_msgs::TransformStamped thermal_realsense_transform;
        try {
            thermal_realsense_transform = tf_buffer.lookupTransform("flir_boson_optical_frame", latest_pointcloud.header.frame_id, latest_pointcloud.header.stamp);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Do transform
        pcl_ros::transformPointCloud("flir_boson_optical_frame", thermal_realsense_transform.transform, latest_pointcloud, transformed_points);

        std::vector<cv::Point2d> projected_points;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points_pcl(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_points, *transformed_points_pcl);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZ>);
        projectPointsToImage(transformed_points_pcl, camera_matrix, dist_coeffs, projected_points);
        filterPointsByMask(projected_points, transformed_points_pcl, filtered_points);

        if (!filtered_points->points.empty()) {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*filtered_points, centroid);
            ROS_INFO("Centroid of filtered points: (%f, %f, %f)", centroid[0], centroid[1], centroid[2]);
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_points, output);
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "flir_boson_optical_frame";
        fire_pointcloud_pub.publish(output);
    }

    void filterPointsByMask(const std::vector<cv::Point2d>& projected_points, const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_points) {
        for (size_t i = 0; i < projected_points.size(); ++i) {
            int x = static_cast<int>(projected_points[i].x);
            int y = static_cast<int>(projected_points[i].y);
            if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows && mask.at<uchar>(y, x) > 0) {
                filtered_points->points.push_back(points->points[i]);
            }
        }
    }

    void projectPointsToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, std::vector<cv::Point2d>& projected_points) {
        std::vector<cv::Point3d> points_3d;
        for (const auto& point : points->points) {
            points_3d.push_back(cv::Point3d(point.x, point.y, point.z));
        }
        cv::projectPoints(points_3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix, dist_coeffs, projected_points);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_fire_localization");
    RealSenseFireLocalization realsense_fire_localization;
    ros::spin();
    return 0;
}
