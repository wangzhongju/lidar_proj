#include "utils.h"
extern bool exit_flag;
class DataPreprocess {
public:
    DataPreprocess();
    void init(ros::NodeHandle& nh, const std::string& config_file);
    void start(const std::string& data_path, const std::string& save_path);
    void stop();

private:
    void subNode();
    void pubNode(const std::string& data_path, const std::string& save_path);
    void clipParser(const boost::filesystem::path& bag_file, const std::string& save_path);
    void frameCallback(std::vector<sensor_msgs::PointCloud2ConstPtr> pointcloud_msgs);
    void callbacksingle(const sensor_msgs::PointCloud2ConstPtr& cloud1);
    void callbackdouble(const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2);
    void callbacktriple(const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2, const sensor_msgs::PointCloud2ConstPtr& cloud3);
    void processPointCloud(const std::string& topic, const sensor_msgs::PointCloud2ConstPtr& pc_msg);

private:
    std::map<std::string, Eigen::Matrix4d> transform_map_;
    std::vector<std::string> topic_list_;
    std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>> pointcloud_subs_;
    ros::Subscriber pointcloud_sub_single;
    std::map<std::string, ros::Publisher> pointcloud_pubs_;

private:
    ros::NodeHandle nh_;
    std::string save_path_;
    int frame_;
    int step_;
};
