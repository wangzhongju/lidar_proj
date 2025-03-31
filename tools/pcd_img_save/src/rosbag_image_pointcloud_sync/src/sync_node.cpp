#include "sync_node.h"
#include <sys/stat.h>
#include <ctime>

SyncNode::SyncNode(ros::NodeHandle& nh) : nh_(nh), last_save_time_(0) {
    // 获取参数
    nh_.param("save_interval", save_interval_, 1.0);
    nh_.param("output_directory", output_directory_, std::string("/tmp/sync_output"));

    // 获取多个图像话题和点云话题
    nh_.getParam("image_topics", image_topics_);
    nh_.getParam("pointcloud_topics", pointcloud_topics_);

    // 检查并创建输出目录
    if (mkdir(output_directory_.c_str(), 0777) == -1 && errno != EEXIST) {
        ROS_ERROR("Failed to create output directory: %s", output_directory_.c_str());
    }
    // 为每个图像和点云话题创建订阅器
    for (const auto& topic : image_topics_) {
        image_subs_[topic] = nh_.subscribe<sensor_msgs::CompressedImage>(
            topic, 10, boost::bind(&SyncNode::imageCallback, this, _1, topic));
        ROS_INFO("Subscribed to image topic: %s", topic.c_str());
    }
    for (const auto& topic : pointcloud_topics_) {
        pointcloud_subs_[topic] = nh_.subscribe<sensor_msgs::PointCloud2>(
            topic, 10, boost::bind(&SyncNode::pointcloudCallback, this, _1, topic));
        ROS_INFO("Subscribed to pointcloud topic: %s", topic.c_str());
    }
}

void SyncNode::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg, const std::string& topic) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_images_[topic] = msg;
    checkAndSave();
}

void SyncNode::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string& topic) {
    // 验证点云数据
    if (msg->width == 0 || msg->height == 0) {
        ROS_WARN("Received empty pointcloud on topic %s", topic.c_str());
        return;
    }
    if (msg->data.empty()) {
        ROS_WARN("Received pointcloud with empty data on topic %s", topic.c_str());
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_pointclouds_[topic] = msg;
    checkAndSave();
}

void SyncNode::checkAndSave() {
    ros::Time now = ros::Time::now();
    if ((now.toSec() - last_save_time_) < save_interval_) {
        return;
    }

    // 检查所有图像和点云话题是否都有数据
    if (last_images_.size() != image_topics_.size() || last_pointclouds_.size() != pointcloud_topics_.size()) {
        ROS_DEBUG("Waiting for all topics to have data...");
        return;
    }

    // 为每个图像找到时间最接近的点云
    for (const auto& img_entry : last_images_) {
        const std::string& img_topic = img_entry.first;
        const ros::Time& img_time = img_entry.second->header.stamp;

        double min_diff = std::numeric_limits<double>::max();
        std::string best_pc_topic;
        
        for (const auto& pc_entry : last_pointclouds_) {
            double diff = fabs((img_time - pc_entry.second->header.stamp).toSec());
            if (diff < min_diff) {
                min_diff = diff;
                best_pc_topic = pc_entry.first;
            }
        }

        if (min_diff < save_interval_) {
            saveData(img_topic, best_pc_topic);
        }
    }

    last_save_time_ = now.toSec();
}

std::string SyncNode::extractCameraName(const std::string& topic) {
    std::vector<std::string> parts;
    boost::split(parts, topic, boost::is_any_of("/"));
    if (parts.size() >= 3) {
        return parts[1];
    }
    return "unknown";
}

bool SyncNode::createDirectoryRecursive(const std::string& path) {
    size_t pos = 0;
    std::string dir;
    int mdret;
    
    if (path[path.size()-1] != '/') {
        dir = path + "/";
    } else {
        dir = path;
    }
    
    while ((pos = dir.find_first_of('/', pos)) != std::string::npos) {
        std::string subdir = dir.substr(0, pos);
        if (subdir.empty()) {
            pos++;
            continue;
        }
        
        // 检查目录是否存在
        struct stat st;
        if (stat(subdir.c_str(), &st) == -1) {
            // 目录不存在，创建它
            mdret = mkdir(subdir.c_str(), 0777);
            if (mdret != 0 && errno != EEXIST) {
                ROS_ERROR("Failed to create directory: %s (errno: %d)", subdir.c_str(), errno);
                return false;
            }
        } else if (!S_ISDIR(st.st_mode)) {
            ROS_ERROR("Path exists but is not a directory: %s", subdir.c_str());
            return false;
        }
        pos++;
    }
    
    return true;
}

void SyncNode::saveData(const std::string& image_topic, const std::string& pointcloud_topic) {
    // 1. 验证输入
    if (last_images_.find(image_topic) == last_images_.end() || 
        last_pointclouds_.find(pointcloud_topic) == last_pointclouds_.end()) {
        ROS_ERROR("Attempted to save non-existent data");
        return;
    }

    // 2. 创建目录
    std::string img_cam_name = extractCameraName(image_topic);
    std::string pc_cam_name = extractCameraName(pointcloud_topic);
    
    std::string img_dir = output_directory_ + "/images/" + img_cam_name;
    std::string pc_dir = output_directory_ + "/pointclouds/" + pc_cam_name;

    // if (mkdir(img_dir.c_str(), 0777) == -1 && errno != EEXIST) {
    //     ROS_ERROR("Failed to create image directory: %s", img_dir.c_str());
    //     return;
    // }
    // if (mkdir(pc_dir.c_str(), 0777) == -1 && errno != EEXIST) {
    //     ROS_ERROR("Failed to create pointcloud directory: %s", pc_dir.c_str());
    //     return;
    // }

    // 递归创建目录
    if (!createDirectoryRecursive(img_dir)) {
        ROS_ERROR("Failed to create image directory: %s", img_dir.c_str());
        return;
    }
    if (!createDirectoryRecursive(pc_dir)) {
        ROS_ERROR("Failed to create pointcloud directory: %s", pc_dir.c_str());
        return;
    }

    // 3. 保存点云
    ros::Time pointcloud_time = last_pointclouds_[pointcloud_topic]->header.stamp;
    try {
        auto pc_msg = last_pointclouds_[pointcloud_topic];
        
        // 详细日志记录点云信息
        ROS_DEBUG("Saving pointcloud from topic %s with stamp %f",
                 pointcloud_topic.c_str(), pc_msg->header.stamp.toSec());
        
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*pc_msg, cloud);
        // 检查转换后的点云是否有效
        if (cloud.empty()) {
            ROS_WARN("Converted pointcloud is empty");
            return;
        }
        // if (!pcl::fromROSMsg(*pc_msg, cloud)) {
        //     ROS_ERROR("PointCloud2 to PCL conversion failed");
        //     return;
        // }

        // 移除无效点
        cloud.is_dense = false;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloud, cloud, indices);
        
        if (cloud.empty()) {
            ROS_WARN("Pointcloud after NaN removal is empty");
            return;
        }

        std::string pc_filename = pc_dir + "/" + 
                                 std::to_string(pointcloud_time.toSec()) + ".pcd";
        
        if (pcl::io::savePCDFileBinary(pc_filename, cloud) != 0) {
            ROS_ERROR("PCD file save failed for %s", pc_filename.c_str());
            return;
        }
        
        ROS_INFO("Successfully saved pointcloud: %s", pc_filename.c_str());
    } catch (const std::exception& e) {
        ROS_ERROR("Pointcloud save exception: %s", e.what());
        return;
    }

    // 保存图像
    try {
        auto img_msg = last_images_[image_topic];
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        
        // std::ostringstream img_filename;
        // img_filename << img_dir << "/" 
        //             << std::setfill('0') << std::setw(10) 
        //             << (int)img_msg->header.stamp.toSec() 
        //             << "_" << std::setw(9) 
        //             << (int)(img_msg->header.stamp.toNSec() % 1000000000) 
        //             << ".png";
        // cv::imwrite(img_filename.str(), cv_ptr->image);
        // ROS_INFO("Saved image: %s", img_filename.str().c_str());

        std::string img_filename = img_dir + "/" + std::to_string(pointcloud_time.toSec()) + ".png";
        cv::imwrite(img_filename, cv_ptr->image);
        ROS_INFO("Successfully Saved image: %s", img_filename.c_str());
    } catch (const std::exception& e) {
        ROS_ERROR("Image save error: %s", e.what());
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh("~");

    // // 检查必要参数是否存在
    // if (!nh.hasParam("image_topics") || !nh.hasParam("pointcloud_topics")) {
    //     ROS_ERROR("Missing required parameters: image_topics or pointcloud_topics");
    //     return 1;
    // }

    SyncNode sync_node(nh);
    
    ROS_INFO("Sync node started");
    ros::spin();
    
    return 0;
}