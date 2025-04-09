#include <ros/ros.h>

#include "data_preprocess.h"
// 信号处理函数
void signalHandler(int signum) {
    ROS_INFO("Interrupt signal (%d) received.", signum);
    exit_flag = true;  // 设置退出标志
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "data_preprocess");
    if (argc != 3) {
        ROS_ERROR("Usage: %s <data_path>", argv[1]);
        ROS_ERROR("Usage: %s <save_path>", argv[2]);
        return 1;
    }
    // step 1.加载配置文件路径
    std::experimental::filesystem::path config_path(PROJECT_PATH);
    std::string config_file = config_path.string() + "/config/config.yaml";
    std::string data_path = argv[1];
    std::string save_path = argv[2];
    ros::NodeHandle nh;
    DataPreprocess data_preprocess;
    data_preprocess.init(nh, config_file);
    data_preprocess.start(data_path, save_path);
    ros::spin();
}