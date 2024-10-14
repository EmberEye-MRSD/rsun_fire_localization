// #include <ros/ros.h>
// #include <opencv2/opencv.hpp>
// #include <Eigen/Dense>

// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/point_cloud2_iterator.h>
// #include <geometry_msgs/TransformStamped.h>

// #include <cv_bridge/cv_bridge.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// #include <pcl_ros/transforms.h>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/common/centroid.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/point_types_conversion.h>

// #include <random>

// class RealSenseFireLocalization
// {
// private:
//     ros::NodeHandle nh;

//     ros::Publisher fire_pointcloud_pub;
//     ros::Publisher flir_pointcloud_pub;
//     ros::Publisher cluster_pub;

//     ros::Subscriber depth_sub;
//     ros::Subscriber thermal_image_sub;

//     sensor_msgs::PointCloud2 transformed_points;

//     cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 408.87350878, 0, 309.32990709,
//                                                        0, 408.94608921, 240.2088536,
//                                                        0, 0, 1);

//     cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.402814957, 0.215711407, -0.0000941079706, -0.000224618698, -0.0648024124);

//     cv::Mat mask;

//     // PCL-related variables
//     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points;
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     std::vector<pcl::PointIndices> cluster_indices;

//     // Random number generator for colors
//     std::random_device rd;
//     std::mt19937 gen;
//     std::uniform_int_distribution<> dis;

//     tf2_ros::Buffer tf_buffer;
//     tf2_ros::TransformListener tfListener;

// public:
//     RealSenseFireLocalization() 
//         : filtered_points(new pcl::PointCloud<pcl::PointXYZ>()),
//           tree(new pcl::search::KdTree<pcl::PointXYZ>()),
//           gen(rd()), 
//           dis(0, 255), 
//           tfListener(tf_buffer)
//     {
//         // Initialize ROS publishers and subscribers
//         fire_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/fire_point_cloud", 1);
//         flir_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/flir/points", 1);
//         cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster_points", 1);

//         depth_sub = nh.subscribe("/camera/depth/color/points", 1, &RealSenseFireLocalization::pointCloudCallback, this);
//         thermal_image_sub = nh.subscribe("/flir_boson/image_raw", 1, &RealSenseFireLocalization::thermalImageCallback, this);

//         // Initialize the Euclidean Cluster Extraction
//         ec.setClusterTolerance(0.6); // 60 cm radius
//         ec.setMinClusterSize(5);
//         ec.setMaxClusterSize(25000);
//         ec.setSearchMethod(tree);
//     }

//     void thermalImageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         try {
//             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
//             cv::threshold(cv_ptr->image, mask, 23000, 255, cv::THRESH_BINARY);
//             mask.convertTo(mask, CV_8U);
//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }

//     void filterPointsByMask(const std::vector<cv::Point2d>& projected_points, const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_points) {
//         for (size_t i = 0; i < projected_points.size(); ++i) {
//             int x = static_cast<int>(projected_points[i].x);
//             int y = static_cast<int>(projected_points[i].y);
//             if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows && mask.at<uchar>(y, x) > 0) {
//                 filtered_points->points.push_back(points->points[i]);
//             }
//         }
//     }

//     void projectPointsToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, std::vector<cv::Point2d>& projected_points) {
//         std::vector<cv::Point3d> points_3d;
//         for (const auto& point : points->points) {
//             points_3d.push_back(cv::Point3d(point.x, point.y, point.z));
//         }
//         cv::projectPoints(points_3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix, dist_coeffs, projected_points);
//     }

//     void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
//         if (mask.empty()) return;

//         // Transform the pointcloud from camera_depth_optical_frame to thermal_optical_frame
//         geometry_msgs::TransformStamped thermal_realsense_transform;
//         try {
//             thermal_realsense_transform = tf_buffer.lookupTransform("flir_boson_optical_frame", msg->header.frame_id, ros::Time(0));
//         } catch (tf2::TransformException &ex) {
//             ROS_WARN("%s", ex.what());
//             return;
//         }

//         // Transform point cloud
//         pcl_ros::transformPointCloud("flir_boson_optical_frame", thermal_realsense_transform.transform, *msg, transformed_points);

//         std::vector<cv::Point2d> projected_points;

//         pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points_pcl(new pcl::PointCloud<pcl::PointXYZ>());
//         pcl::fromROSMsg(transformed_points, *transformed_points_pcl);
        
//         projectPointsToImage(transformed_points_pcl, camera_matrix, dist_coeffs, projected_points);

//         filterPointsByMask(projected_points, transformed_points_pcl, filtered_points);

//         if (!filtered_points->points.empty()) {
//             Eigen::Vector4f centroid;
//             pcl::compute3DCentroid(*filtered_points, centroid);
//             ROS_INFO("Centroid of filtered points: (%f, %f, %f)", centroid[0], centroid[1], centroid[2]);
//         }

//         sensor_msgs::PointCloud2 output;
//         pcl::toROSMsg(*filtered_points, output);
//         output.header.stamp = ros::Time::now();
//         output.header.frame_id = "flir_boson_optical_frame";
//         fire_pointcloud_pub.publish(output);

//         // Cluster the points and publish the clusters
//         // tree->setInputCloud(filtered_points);
//         ec.setInputCloud(filtered_points);
//         ec.extract(cluster_indices);

//         for (const auto& indices : cluster_indices) {
//             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
//             uint8_t r = static_cast<uint8_t>(dis(gen));
//             uint8_t g = static_cast<uint8_t>(dis(gen));
//             uint8_t b = static_cast<uint8_t>(dis(gen));

//             for (const auto& idx : indices.indices) {
//                 pcl::PointXYZRGB point;
//                 point.x = filtered_points->points[idx].x;
//                 point.y = filtered_points->points[idx].y;
//                 point.z = filtered_points->points[idx].z;
//                 point.r = r;
//                 point.g = g;
//                 point.b = b;
//                 cluster->points.push_back(point);
//             }

//             cluster->width = cluster->points.size();
//             cluster->height = 1;
//             cluster->is_dense = true;

//             // Publish each cluster separately
//             sensor_msgs::PointCloud2 cluster_output;
//             pcl::toROSMsg(*cluster, cluster_output);
//             cluster_output.header.frame_id = "flir_boson_optical_frame";
//             cluster_output.header.stamp = ros::Time::now();
//             cluster_pub.publish(cluster_output);
//         }

//         std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;

//         ec.setInputCloud(nullptr);
//     }
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "realsense_fire_localization");
//     RealSenseFireLocalization realsense_fire_localization;
//     ros::spin();
//     return 0;
// }









// #include <ros/ros.h>
// #include <opencv2/opencv.hpp>
// #include <Eigen/Dense>

// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/point_cloud2_iterator.h>
// #include <geometry_msgs/TransformStamped.h>

// #include <cv_bridge/cv_bridge.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// #include <pcl_ros/transforms.h>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/common/centroid.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/point_types_conversion.h>

// #include <random>

// class RealSenseFireLocalization
// {
// private:
//     ros::NodeHandle nh;

//     ros::Publisher fire_pointcloud_pub;
//     ros::Publisher flir_pointcloud_pub;
    
//     // for viz of clusters and hotspots
//     ros::Publisher cluster_mask_pub1;
//     ros::Publisher cluster_mask_pub2;

//     ros::Publisher fire_pointcloud_pub1;
//     ros::Publisher fire_pointcloud_pub2;

//     ros::Subscriber depth_sub;
//     ros::Subscriber thermal_image_sub;

//     sensor_msgs::PointCloud2 transformed_points;

//     cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 415.68148228, 0, 312.98599158,
//                                                        0, 415.74859039, 254.63140201,
//                                                        0, 0, 1);

//     cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -4.16123804e-01, 2.33440434e-01, -1.19852069e-04, 5.89824552e-04, -7.49109237e-02);

//     cv::Mat mask;

//     // // PCL-related variables
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points;
//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_points;
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     std::vector<pcl::PointIndices> cluster_indices;

//     // Multi hotspot localization
//     // std::vector<std::vector<cv::Point2d>> mask_clusters;
//     std::vector<cv::Mat> mask_clusters;

//     // Random number generator for colors
//     std::random_device rd;
//     std::mt19937 gen;
//     std::uniform_int_distribution<> dis;

//     tf2_ros::Buffer tf_buffer;
//     tf2_ros::TransformListener tfListener;

// public:
//     RealSenseFireLocalization() 
//         : filtered_points(new std::vector<pcl::PointCloud<pcl::PointXYZ>()>),
//           tree(new pcl::search::KdTree<pcl::PointXYZ>()),
//           gen(rd()), 
//           dis(0, 255), 
//           tfListener(tf_buffer)
//     {
//         // Initialize ROS publishers and subscribers
//         fire_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/fire_point_cloud", 1);
//         flir_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/flir/points", 1);

//         // for viz of clusters and hotspots
//         cluster_mask_pub1 = nh.advertise<sensor_msgs::Image>("/cluster_mask1", 1);
//         cluster_mask_pub2 = nh.advertise<sensor_msgs::Image>("/cluster_mask2", 1);

//         fire_pointcloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("/fire_point_cloud1", 1);
//         fire_pointcloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("/fire_point_cloud2", 1);

//         depth_sub = nh.subscribe("/camera/depth/color/points", 1, &RealSenseFireLocalization::pointCloudCallback, this);
//         thermal_image_sub = nh.subscribe("/flir_boson/image_raw", 1, &RealSenseFireLocalization::thermalImageCallback, this);
//     }

//     void thermalImageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         try {
//             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
//             cv::threshold(cv_ptr->image, mask, 23000, 255, cv::THRESH_BINARY);
//             mask.convertTo(mask, CV_8U);

//             mask_clusters.clear();
//             findClusters(mask, mask_clusters);

//             // // Publish the cluster masks
//             // for (size_t i = 0; i < mask_clusters.size(); ++i) {
//             //     cv_bridge::CvImage cv_image;
//             //     cv_image.encoding = "8UC1";
//             //     cv_image.image = mask_clusters[i];
//             //     if (i == 0) {
//             //         cluster_mask_pub1.publish(cv_image.toImageMsg());
//             //     } else {
//             //         cluster_mask_pub2.publish(cv_image.toImageMsg());
//             //     }
//             // }

//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }

//     void filterPointsByMask(const std::vector<cv::Point2d>& projected_points, const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& filtered_points) {
//         for (size_t i = 0; i < projected_points.size(); ++i) {
//             int x = static_cast<int>(projected_points[i].x);
//             int y = static_cast<int>(projected_points[i].y);
//             for (size_t j=0; j<mask_clusters.size(); ++j){
//                 if (x>=0 && x<mask_clusters[j].cols && y>=0 && y<mask_clusters[j].rows && mask_clusters[j].at<uchar>(y, x) > 0){
//                     filtered_points[j]->points.push_back(points->points[i]);
//                 }
//                 continue;
//             }
//             // if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows && mask.at<uchar>(y, x) > 0) {
//             //     filtered_points->points.push_back(points->points[i]);
//             // }
//         }
//     }

//     void findClusters(const cv::Mat& binary_image, std::vector<cv::Mat>& clusters) {
//         // Label the clusters using connected components
//         cv::Mat labels, stats, centroids;
//         int num_labels = cv::connectedComponentsWithStats(binary_image, labels, stats, centroids);

//         // Iterate over each label (ignoring the background, label 0)
//         for (int label = 1; label < num_labels; ++label) {
//             // Extract the mask for the current label
//             cv::Mat cluster_mask = (labels == label);
            
//             // Remove small clusters
//             int cluster_size = stats.at<int>(label, cv::CC_STAT_AREA);
//             if (cluster_size < 100) {
//                 continue; // Skip this cluster if it's too small
//             }

//             clusters.push_back(cluster_mask.clone());
//         }

//         // Print the number of clusters found
//         std::cout << "Number of clusters: " << clusters.size() << std::endl;

//         // // Compute and print the mean position of each cluster
//         // for (size_t i = 0; i < clusters.size(); ++i) {
//         //     double mean_x = 0;
//         //     double mean_y = 0;
//         //     int count = 0;

//         //     // Iterate through the mask to calculate the mean location of cluster pixels
//         //     for (int row = 0; row < clusters[i].rows; ++row) {
//         //         for (int col = 0; col < clusters[i].cols; ++col) {
//         //             if (clusters[i].at<uchar>(row, col) > 0) {
//         //                 mean_x += col;
//         //                 mean_y += row;
//         //                 ++count;
//         //             }
//         //         }
//         //     }

//         //     if (count > 0) {
//         //         mean_x /= count;
//         //         mean_y /= count;
//         //     }

//         //     std::cout << "Cluster " << i + 1 << " has mean: (" << mean_x << ", " << mean_y << ")" << std::endl;
//         // }
//     }


//     void projectPointsToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, std::vector<cv::Point2d>& projected_points) {
//         std::vector<cv::Point3d> points_3d;
//         for (const auto& point : points->points) {
//             points_3d.push_back(cv::Point3d(point.x, point.y, point.z));
//         }
//         cv::projectPoints(points_3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix, dist_coeffs, projected_points);
//     }

//     void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
//         if (mask.empty()) return;

//         // Transform the pointcloud from camera_depth_optical_frame to thermal_optical_frame
//         geometry_msgs::TransformStamped thermal_realsense_transform;
//         try {
//             thermal_realsense_transform = tf_buffer.lookupTransform("flir_boson_optical_frame", msg->header.frame_id, ros::Time(0));
//         } catch (tf2::TransformException &ex) {
//             ROS_WARN("%s", ex.what());
//             return;
//         }

//         // Transform point cloud
//         pcl_ros::transformPointCloud("flir_boson_optical_frame", thermal_realsense_transform.transform, *msg, transformed_points);

//         std::vector<cv::Point2d> projected_points;

//         pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points_pcl(new pcl::PointCloud<pcl::PointXYZ>());
//         pcl::fromROSMsg(transformed_points, *transformed_points_pcl);
        
//         // Clear filtered points before adding new points
//         filtered_points.clear();

//         projectPointsToImage(transformed_points_pcl, camera_matrix, dist_coeffs, projected_points);

//         filterPointsByMask(projected_points, transformed_points_pcl, filtered_points);

//         if (filtered_points.size() != 0){
//             for (size_t i=0; i<filtered_points.size(); ++i){
//                 if (!filtered_points[i]->points.empty()) {
//                     Eigen::Vector4f centroid;
//                     pcl::compute3DCentroid(*filtered_points[i], centroid);
//                     ROS_INFO("Centroid of filtered points (%zu): (%f, %f, %f)", i, centroid[0], centroid[1], centroid[2]);
//                 }

//                 sensor_msgs::PointCloud2 output;
//                 pcl::toROSMsg(*filtered_points[i], output);
//                 output.header.stamp = ros::Time::now();
//                 output.header.frame_id = "flir_boson_optical_frame";

//                 if (i == 0){
//                     fire_pointcloud_pub1.publish(output);
//                 } else {
//                     fire_pointcloud_pub2.publish(output);
//                 }
//             }
//         }

//         // if (!filtered_points->points.empty()) {
//         //     Eigen::Vector4f centroid;
//         //     pcl::compute3DCentroid(*filtered_points, centroid);
//         //     // ROS_INFO("Centroid of filtered points: (%f, %f, %f)", centroid[0], centroid[1], centroid[2]);
//         // }

//         // sensor_msgs::PointCloud2 output;
//         // pcl::toROSMsg(*filtered_points, output);
//         // output.header.stamp = ros::Time::now();
//         // output.header.frame_id = "flir_boson_optical_frame";
//         // fire_pointcloud_pub.publish(output);
//     }
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "realsense_fire_localization");
//     RealSenseFireLocalization realsense_fire_localization;
//     ros::spin();
//     return 0;
// }



#include <vector>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>

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

#include <random>

class RealSenseFireLocalization
{
private:
    ros::NodeHandle nh;

    ros::Publisher fire_pointcloud_pub;
    ros::Publisher flir_pointcloud_pub;
    
    // for viz of clusters and hotspots
    ros::Publisher cluster_mask_pub1;
    ros::Publisher cluster_mask_pub2;

    ros::Publisher fire_pointcloud_pub1;
    ros::Publisher fire_pointcloud_pub2;

    ros::Subscriber depth_sub;
    ros::Subscriber thermal_image_sub;

    sensor_msgs::PointCloud2 transformed_points;

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 415.68148228, 0, 312.98599158,
                                                       0, 415.74859039, 254.63140201,
                                                       0, 0, 1);

    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -4.16123804e-01, 2.33440434e-01, -1.19852069e-04, 5.89824552e-04, -7.49109237e-02);

    cv::Mat mask;

    // PCL-related variables
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_points;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    std::vector<pcl::PointIndices> cluster_indices;

    // Multi hotspot localization
    std::vector<cv::Mat> mask_clusters;

    // Random number generator for colors
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<> dis;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfListener;

public:
    RealSenseFireLocalization() 
        : tree(new pcl::search::KdTree<pcl::PointXYZ>()),
          gen(rd()), 
          dis(0, 255), 
          tfListener(tf_buffer)
    {
        // Initialize ROS publishers and subscribers
        fire_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/fire_point_cloud", 1);
        flir_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/flir/points", 1);

        // for viz of clusters and hotspots
        cluster_mask_pub1 = nh.advertise<sensor_msgs::Image>("/cluster_mask1", 1);
        cluster_mask_pub2 = nh.advertise<sensor_msgs::Image>("/cluster_mask2", 1);

        fire_pointcloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("/fire_point_cloud1", 1);
        fire_pointcloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("/fire_point_cloud2", 1);

        depth_sub = nh.subscribe("/camera/depth/color/points", 1, &RealSenseFireLocalization::pointCloudCallback, this);
        thermal_image_sub = nh.subscribe("/flir_boson/image_raw", 1, &RealSenseFireLocalization::thermalImageCallback, this);
    }

    void thermalImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
            cv::threshold(cv_ptr->image, mask, 23000, 255, cv::THRESH_BINARY);
            mask.convertTo(mask, CV_8U);

            mask_clusters.clear();
            findClusters(mask, mask_clusters);

            // Publish the cluster masks
            // for (size_t i = 0; i < mask_clusters.size(); ++i) {
            //     cv_bridge::CvImage cv_image;
            //     cv_image.encoding = "8UC1";
            //     cv_image.image = mask_clusters[i];
            //     if (i == 0) {
            //         cluster_mask_pub1.publish(cv_image.toImageMsg());
            //     } else {
            //         cluster_mask_pub2.publish(cv_image.toImageMsg());
            //     }
            // }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    // void filterPointsByMask(const std::vector<cv::Point2d>& projected_points, const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& filtered_points) {
    //     filtered_points.clear();
    //     filtered_points.resize(mask_clusters.size(), pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()));

    //     // Publish the cluster masks
    //     for (size_t i = 0; i < mask_clusters.size(); ++i) {
    //         cv_bridge::CvImage cv_image;
    //         cv_image.encoding = "8UC1";
    //         cv_image.image = mask_clusters[i];
    //         if (i == 0) {
    //             cluster_mask_pub1.publish(cv_image.toImageMsg());
    //         } else {
    //             cluster_mask_pub2.publish(cv_image.toImageMsg());
    //         }
    //     }

    //     for (size_t i = 0; i < projected_points.size(); ++i) {
    //         int x = static_cast<int>(projected_points[i].x);
    //         int y = static_cast<int>(projected_points[i].y);
    //         for (size_t j = 0; j < mask_clusters.size(); ++j) {
    //             if (x >= 0 && x < mask_clusters[j].cols && y >= 0 && y < mask_clusters[j].rows && mask_clusters[j].at<uchar>(y, x) > 0) {
    //                 filtered_points[j]->points.push_back(points->points[i]);
    //                 break;
    //             }
    //         }
    //     }

    //     for (size_t i=0; i<filtered_points.size(); ++i){
    //         std::cout << "Number of points in cluster " << i << ": " << filtered_points[i]->points.size() << std::endl;
    //     }


    // }

    void filterPointsByMask(const std::vector<cv::Point2d>& projected_points, const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& filtered_points) {
        // Clear the previous points and resize the vector to match the number of clusters
        filtered_points.clear();
        filtered_points.resize(mask_clusters.size());

        // Initialize each filtered point cloud pointer
        for (size_t i = 0; i < filtered_points.size(); ++i) {
            filtered_points[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        }

        // Publish the cluster masks
        for (size_t i = 0; i < mask_clusters.size(); ++i) {
            cv_bridge::CvImage cv_image;
            cv_image.encoding = "8UC1";
            cv_image.image = mask_clusters[i];
            if (i == 0) {
                cluster_mask_pub1.publish(cv_image.toImageMsg());
            } else {
                cluster_mask_pub2.publish(cv_image.toImageMsg());
            }
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

        // Print the number of points in each cluster
        for (size_t i = 0; i < filtered_points.size(); ++i) {
            std::cout << "Number of points in cluster " << i << ": " << filtered_points[i]->points.size() << std::endl;
        }
    }


    void findClusters(const cv::Mat& binary_image, std::vector<cv::Mat>& clusters) {
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

        // Print the number of clusters found
        // std::cout << "Number of clusters: " << clusters.size() << std::endl;
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

                // print the number of points in each cluster
                // std::cout << "Number of points in cluster " << i << ": " << filtered_points[i]->points.size() << std::endl;
                // std::cout << "cluster " << i << std::endl;

                sensor_msgs::PointCloud2 output;
                pcl::toROSMsg(*filtered_points[i], output);
                output.header.stamp = ros::Time::now();
                output.header.frame_id = "flir_boson_optical_frame";

                if (i == 0) {
                    fire_pointcloud_pub1.publish(output);
                } else {
                    fire_pointcloud_pub2.publish(output);
                }
            }

            std::cout << std::endl;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_fire_localization");
    RealSenseFireLocalization realsense_fire_localization;
    ros::spin();
    return 0;
}