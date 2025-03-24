#include "ros/ros.h"
#include "my_rtsp_streamer/rtsp_streamer.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "rtsp_streamer_node");
    ros::NodeHandle nh("~");

    std::string config_file;
    nh.getParam("config_file", config_file);

    my_rtsp_streamer::RTSPStreamer rtsp_streamer(nh, config_file);

    ros::spin();
    return 0;
}