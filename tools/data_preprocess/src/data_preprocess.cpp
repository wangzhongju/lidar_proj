#include "data_preprocess.h"
bool exit_flag = false;
DataPreprocess::DataPreprocess() {}

void DataPreprocess::init(ros::NodeHandle& nh, const std::string& config_path) {
    // 初始化
    this->nh_ = nh;
    YAML::Node config = YAML::LoadFile(config_path);
    YAML::Node topic_list = config["topic_list"];
    step_ = config["step"].as<int>();
    for (auto topic : topic_list) {
        std::string topic_name = topic["name"].as<std::string>();
        double x = topic["transform"]["x"].as<double>();
        double y = topic["transform"]["y"].as<double>();
        double z = topic["transform"]["z"].as<double>();
        double roll = topic["transform"]["roll"].as<double>();
        double pitch = topic["transform"]["pitch"].as<double>();
        double yaw = topic["transform"]["yaw"].as<double>();
        Eigen::Matrix4d transform = getTransform(x, y, z, roll, pitch, yaw);
        transform_map_[topic_name] = transform;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pointcloud_sub =
            std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, topic_name, 1);
        pointcloud_subs_.push_back(pointcloud_sub);
        pointcloud_pubs_[topic_name] = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 10);
        topic_list_.push_back(topic_name);
    }
    exit_flag = false;
}

void DataPreprocess::start(const std::string& data_path, const std::string& save_path) {
    std::thread pub_thread(&DataPreprocess::pubNode, this, data_path, save_path);
    std::thread sub_thread(&DataPreprocess::subNode, this);
    pub_thread.join();
    sub_thread.join();
}
void DataPreprocess::subNode() {
    // 如果有点云订阅者，创建同步器
    if (!pointcloud_subs_.empty()) {
        // 根据订阅的点云数量设置同步器
        if (pointcloud_subs_.size() == 1) {
            pointcloud_sub_single = nh_.subscribe(pointcloud_subs_[0]->getTopic(), 10, &DataPreprocess::callbacksingle, this);
            ros::Rate rate(100);  // 设置循环频率，例如 10 Hz
            while (ros::ok() && !exit_flag) {
                ros::spinOnce();  // 处理回调
                rate.sleep();     // 控制循环频率
            }
        } else if (pointcloud_subs_.size() == 2) {
            message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>> sync_double_(
                message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>(10), *pointcloud_subs_[0], *pointcloud_subs_[1]);
            sync_double_.registerCallback(boost::bind(&DataPreprocess::callbackdouble, this, _1, _2));
            ros::Rate rate(100);  // 设置循环频率，例如 10 Hz
            while (ros::ok() && !exit_flag) {
                ros::spinOnce();  // 处理回调
                rate.sleep();     // 控制循环频率
            }
        } else if (pointcloud_subs_.size() == 3) {
            message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>>
                sync_triple_(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>(10),
                             *pointcloud_subs_[0], *pointcloud_subs_[1], *pointcloud_subs_[2]);
            sync_triple_.registerCallback(boost::bind(&DataPreprocess::callbacktriple, this, _1, _2, _3));
            ros::Rate rate(100);  // 设置循环频率，例如 10 Hz
            while (ros::ok() && !exit_flag) {
                ros::spinOnce();  // 处理回调
                rate.sleep();     // 控制循环频率
            }
        } else {
            // 如果有更多的点云订阅者，可以添加更多的条件判断，支持更多的点云话题同步
            ROS_WARN("Unsupported number of point cloud topics");
        }
    }
}
void DataPreprocess::pubNode(const std::string& data_path, const std::string& save_path) {
    auto bag_files = loadBagFiles(data_path);
    if (!boost::filesystem::exists(save_path)) {
        boost::filesystem::create_directory(save_path);
    }
    size_t total_files = bag_files.size();  // 获取总文件数
    for (size_t i = 0; i < total_files; ++i) {
        printProgress(i, total_files);
        clipParser(bag_files[i], save_path);
    }
    exit_flag = true;
    ROS_INFO_STREAM("Data parser finished");
    exit(0);
}
void DataPreprocess::clipParser(const boost::filesystem::path& bag_file, const std::string& save_path) {
    std::string bag_file_name = boost::filesystem::basename(bag_file);
    save_path_ = save_path + "/" + bag_file_name;
    frame_ = 0;
    if (boost::filesystem::exists(save_path_)) {
        boost::filesystem::remove_all(save_path_);
    }
    boost::filesystem::create_directory(save_path_);
    for (auto topic : topic_list_) {
        std::vector<std::string> topic_name = extractBetweenSlashes(topic);
        std::string topic_save_path = save_path_ + "/" + topic_name[0];
        boost::filesystem::create_directory(topic_save_path);
    }
    boost::filesystem::create_directory(save_path_ + "/merge");

    rosbag::Bag bag;
    try {
        // 打开 bag 文件
        ROS_INFO_STREAM("Opening bag file: " << bag_file.string());
        bag.open(bag_file.string(), rosbag::bagmode::Read);

        rosbag::View view(bag);

        // 遍历每条消息并处理
        for (rosbag::MessageInstance const& msg_instance : view) {
            // 获取消息类型
            const std::string topic = msg_instance.getTopic();
            ros::Duration(0.1).sleep();
            // 根据消息类型做不同的处理
            if (msg_instance.isType<sensor_msgs::PointCloud2>()) {
                sensor_msgs::PointCloud2::ConstPtr pc_msg = msg_instance.instantiate<sensor_msgs::PointCloud2>();
                if (pc_msg != nullptr) {
                    // 在这里处理点云消息，做一些自定义操作
                    processPointCloud(topic, pc_msg);  // 自定义点云处理函数
                }
            }
        }

    } catch (const rosbag::BagException& e) {
        ROS_ERROR_STREAM("Error opening bag file " << bag_file << ": " << e.what());
    }
    bag.close();
}
void DataPreprocess::processPointCloud(const std::string& topic, const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
    if (transform_map_.find(topic) != transform_map_.end()) {
        pointcloud_pubs_[topic].publish(pc_msg);
    }
}
void DataPreprocess::callbacksingle(const sensor_msgs::PointCloud2ConstPtr& cloud1) {
    if (frame_ % step_ == 0) {
        frame_++;
    } else {
        frame_++;
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud1, *pcl_cloud1);
    std::string topic_name = this->topic_list_[0];
    std::vector<std::string> topic_name_list = extractBetweenSlashes(topic_name);
    Eigen::Matrix4d first_transform = transform_map_[topic_name];
    pcl::transformPointCloud(*pcl_cloud1, *pcl_cloud1, first_transform);
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *merged_cloud = *pcl_cloud1;
    std::string save_path = save_path_ + "/" + topic_name_list[0] + "/" + std::to_string(cloud1->header.stamp.toNSec()) + ".pcd";
    std::string merged_save_path = save_path_ + "/" + "merge" + "/" + std::to_string(cloud1->header.stamp.toNSec()) + ".pcd";
    if (boost::filesystem::exists(save_path)) {
        boost::filesystem::remove(save_path);
    }
    pcl::io::savePCDFileBinaryCompressed(save_path, *pcl_cloud1);
    if (boost::filesystem::exists(merged_save_path)) {
        boost::filesystem::remove(merged_save_path);
    }
    pcl::io::savePCDFileBinaryCompressed(merged_save_path, *merged_cloud);
}
void DataPreprocess::callbackdouble(const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2) {
    if (frame_ % step_ == 0) {
        frame_++;

    } else {
        frame_++;
        return;
    }
    // 可以将点云数据转换为 PCL 格式，方便后续处理
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud2(new pcl::PointCloud<pcl::PointXYZI>());

    // 将 PointCloud2 消息转换为 PCL 点云格式
    pcl::fromROSMsg(*cloud1, *pcl_cloud1);
    pcl::fromROSMsg(*cloud2, *pcl_cloud2);

    // 这里可以对 pcl_cloud1 和 pcl_cloud2 进行任何处理，比如滤波、配准等
    std::string first_topic_name = this->topic_list_[0];
    std::vector<std::string> first_topic_name_list = extractBetweenSlashes(first_topic_name);
    std::string second_topic_name = this->topic_list_[1];
    std::vector<std::string> second_topic_name_list = extractBetweenSlashes(second_topic_name);
    Eigen::Matrix4d first_transform = transform_map_[first_topic_name];
    Eigen::Matrix4d second_transform = transform_map_[second_topic_name];
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud(*pcl_cloud1, *pcl_cloud1, first_transform);
    pcl::transformPointCloud(*pcl_cloud2, *pcl_cloud2, second_transform);
    *merged_cloud = *pcl_cloud1 + *pcl_cloud2;
    std::string first_save_path = save_path_ + "/" + first_topic_name_list[0] + "/" + std::to_string(cloud1->header.stamp.toNSec()) + ".pcd";
    std::string second_save_path = save_path_ + "/" + second_topic_name_list[0] + "/" + std::to_string(cloud2->header.stamp.toNSec()) + ".pcd";
    std::string merged_save_path = save_path_ + "/" + "merge" + "/" + std::to_string(cloud1->header.stamp.toNSec()) + ".pcd";
    if (boost::filesystem::exists(first_save_path)) {
        boost::filesystem::remove(first_save_path);
    }
    pcl::io::savePCDFileBinaryCompressed(first_save_path, *pcl_cloud1);
    if (boost::filesystem::exists(second_save_path)) {
        boost::filesystem::remove(second_save_path);
    }
    pcl::io::savePCDFileBinaryCompressed(second_save_path, *pcl_cloud2);
    if (boost::filesystem::exists(merged_save_path)) {
        boost::filesystem::remove(merged_save_path);
    }
    pcl::io::savePCDFileBinaryCompressed(merged_save_path, *merged_cloud);
}

void DataPreprocess::callbacktriple(const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2,
                                    const sensor_msgs::PointCloud2ConstPtr& cloud3) {
    if (frame_ % step_ == 0) {
        frame_++;

    } else {
        frame_++;
        return;
    }

    // 可以将点云数据转换为 PCL 格式，方便后续处理
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud3(new pcl::PointCloud<pcl::PointXYZI>());

    // 将 PointCloud2 消息转换为 PCL 点云格式
    pcl::fromROSMsg(*cloud1, *pcl_cloud1);
    pcl::fromROSMsg(*cloud2, *pcl_cloud2);
    pcl::fromROSMsg(*cloud3, *pcl_cloud3);

    // 这里可以对 pcl_cloud1 和 pcl_cloud2 进行任何处理，比如滤波、配准等
    std::string first_topic_name = this->topic_list_[0];
    std::string second_topic_name = this->topic_list_[1];
    std::string third_topic_name = this->topic_list_[2];
    std::vector<std::string> first_topic_name_list = extractBetweenSlashes(first_topic_name);
    std::vector<std::string> second_topic_name_list = extractBetweenSlashes(second_topic_name);
    std::vector<std::string> third_topic_name_list = extractBetweenSlashes(third_topic_name);
    Eigen::Matrix4d first_transform = transform_map_[first_topic_name];
    Eigen::Matrix4d second_transform = transform_map_[second_topic_name];
    Eigen::Matrix4d third_transform = transform_map_[third_topic_name];
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud(*pcl_cloud1, *pcl_cloud1, first_transform);
    pcl::transformPointCloud(*pcl_cloud2, *pcl_cloud2, second_transform);
    pcl::transformPointCloud(*pcl_cloud3, *pcl_cloud3, third_transform);
    *merged_cloud = *pcl_cloud1 + *pcl_cloud2 + *pcl_cloud3;
    std::string first_save_path = save_path_ + "/" + first_topic_name_list[0] + "/" + std::to_string(cloud1->header.stamp.toNSec()) + ".pcd";
    std::string second_save_path = save_path_ + "/" + second_topic_name_list[0] + "/" + std::to_string(cloud2->header.stamp.toNSec()) + ".pcd";
    std::string third_save_path = save_path_ + "/" + third_topic_name_list[0] + "/" + std::to_string(cloud3->header.stamp.toNSec()) + ".pcd";
    std::string merged_save_path = save_path_ + "/" + "merge" + "/" + std::to_string(cloud1->header.stamp.toNSec()) + ".pcd";
    if (boost::filesystem::exists(first_save_path)) {
        boost::filesystem::remove(first_save_path);
    }
    pcl::io::savePCDFileBinaryCompressed(first_save_path, *pcl_cloud1);
    if (boost::filesystem::exists(second_save_path)) {
        boost::filesystem::remove(second_save_path);
    }
    pcl::io::savePCDFileBinaryCompressed(second_save_path, *pcl_cloud2);
    if (boost::filesystem::exists(third_save_path)) {
        boost::filesystem::remove(third_save_path);
    }
    pcl::io::savePCDFileBinaryCompressed(third_save_path, *pcl_cloud3);
    if (boost::filesystem::exists(merged_save_path)) {
        boost::filesystem::remove(merged_save_path);
    }
    pcl::io::savePCDFileBinaryCompressed(merged_save_path, *merged_cloud);
}