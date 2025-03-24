#include "my_rtsp_streamer/rtsp_streamer.h"

namespace my_rtsp_streamer {

RTSPStreamer::RTSPStreamer(ros::NodeHandle& nh, const std::string& config_file) : nh_(nh) {
    YAML::Node config = YAML::LoadFile(config_file);
    auto streams = config["streams"];
    for (const auto& stream : streams) {
        std::string name = stream["name"].as<std::string>();
        std::string url = stream["url"].as<std::string>();

        image_transport::ImageTransport it(nh_);
        image_publishers_[name] = it.advertise("/" + name + "/image_raw", 1);

        threads_.emplace_back(std::thread(&RTSPStreamer::streamVideo, this, name, url));
    }
}

RTSPStreamer::~RTSPStreamer() {
    for (auto& thread : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

void RTSPStreamer::streamVideo(const std::string& name, const std::string& url) {
    cv::VideoCapture cap(url);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open RTSP stream: %s", url.c_str());
        return;
    }

    cv_bridge::CvImage cv_image;
    cv_image.encoding = "bgr8";
    ros::Rate rate(30);

    while (ros::ok()) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            ROS_WARN("Failed to read frame from stream: %s", url.c_str());
            break;
        }

        cv_image.header.stamp = ros::Time::now();
        cv_image.image = frame;

        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        image_publishers_[name].publish(ros_image);

        rate.sleep();
    }

    cap.release();
}

} // namespace my_rtsp_streamer
