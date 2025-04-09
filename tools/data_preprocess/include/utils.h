#include <geometry_msgs/PoseStamped.h>
#include <gflags/gflags.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>  // 引入VoxelGrid滤波器
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <string.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <armadillo>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <cmath>
#include <experimental/filesystem>
#include <iostream>
#include <map>
#include <mlpack/methods/gmm/gmm.hpp>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "std_msgs/Header.h"
#include "yaml-cpp/yaml.h"

Eigen::Matrix4d getTransform(double x, double y, double z, double roll, double pitch, double yaw);
std::vector<boost::filesystem::path> loadBagFiles(const std::string& folderPath);
std::vector<std::string> extractBetweenSlashes(const std::string& input);
void printProgress(size_t current, size_t total);