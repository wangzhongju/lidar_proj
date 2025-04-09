#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <ctime>
#include <iomanip>

void convertSecsNsecsToTimestamp(long secs, long nsecs) {
    // 获取时间戳
    std::chrono::seconds sec_duration(secs);
    std::chrono::nanoseconds nsec_duration(nsecs);

    // 将秒数转换为系统时间
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(secs);
    
    // 获取时间结构
    std::time_t tt = std::chrono::system_clock::to_time_t(tp);
    std::tm *gmt = std::gmtime(&tt);  // 获取UTC时间

    // 获取毫秒
    long millisec = nsecs / 1000000;

    // 输出年月日时分秒毫秒格式
    ROS_INFO_STREAM(std::put_time(gmt, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3) << std::setfill('0') << millisec);
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 从消息头中获取时间戳
    long secs = msg->header.stamp.sec;
    long nsecs = msg->header.stamp.nsec;

    // 转换并输出时间戳
    convertSecsNsecsToTimestamp(secs, nsecs);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "timestamp_converter");
    ros::NodeHandle nh;

    // 订阅话题
    ros::Subscriber sub = nh.subscribe("/64/rslidar_points", 10, pointCloudCallback);

    // 循环处理消息
    ros::spin();

    return 0;
}