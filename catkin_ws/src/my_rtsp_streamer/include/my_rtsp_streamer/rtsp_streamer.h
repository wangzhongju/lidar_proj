#ifndef MY_RTSP_STREAMER_RTSP_STREAMER_H
#define MY_RTSP_STREAMER_RTSP_STREAMER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <vector>
#include <unordered_map>

namespace my_rtsp_streamer {

class RTSPStreamer {
public:
    RTSPStreamer(ros::NodeHandle& nh, const std::string& config_file);
    ~RTSPStreamer();

private:
    void streamVideo(const std::string& name, const std::string& url);

    ros::NodeHandle nh_;
    std::unordered_map<std::string, image_transport::Publisher> image_publishers_;
    std::vector<std::thread> threads_;
};

} // namespace my_rtsp_streamer

#endif // MY_RTSP_STREAMER_RTSP_STREAMER_H
