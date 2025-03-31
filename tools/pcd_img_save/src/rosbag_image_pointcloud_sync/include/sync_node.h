#ifndef SYNC_NODE_H
#define SYNC_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h> // 包含removeNaNFromPointCloud
#include <pcl_ros/point_cloud.h>
#include <boost/algorithm/string.hpp>
#include <mutex>
#include <map>

class SyncNode {
public:
    SyncNode(ros::NodeHandle& nh);
    
private:
    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg, const std::string& topic);
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string& topic);
    void checkAndSave();
    void saveData(const std::string& image_topic, const std::string& pointcloud_topic);
    std::string extractCameraName(const std::string& topic);
    bool createDirectoryRecursive(const std::string& path);
    
    ros::NodeHandle nh_;
    std::map<std::string, ros::Subscriber> image_subs_;
    std::map<std::string, ros::Subscriber> pointcloud_subs_;
    std::map<std::string, sensor_msgs::CompressedImageConstPtr> last_images_;
    std::map<std::string, sensor_msgs::PointCloud2ConstPtr> last_pointclouds_;
    
    std::vector<std::string> image_topics_;
    std::vector<std::string> pointcloud_topics_;
    std::string output_directory_;
    double save_interval_;
    double last_save_time_;
    
    std::mutex data_mutex_;
};

#endif // SYNC_NODE_H