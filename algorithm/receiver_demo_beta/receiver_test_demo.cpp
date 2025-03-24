/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include <signal.h>
#include <unistd.h>
#include "rs_application/application.h"
#include "rs_dependence/rs_dependence_manager.h"
#include "rs_perception/communication/external/receiver.h"

#ifdef RS_ROS_FOUND
#include <ros/ros.h>
#endif  // RS_ROS_FOUND

#ifdef RS_ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#endif  // RS_ROS2_FOUND

using namespace robosense;

bool start_ = true;
/**
 * @brief  signal handler
 * @note   will be called if receive ctrl+c signal from keyboard during the progress
 *         (all the threads in progress will be stopped and the progress end)
 * @param  sig: the input signal
 * @retval None
 */
static void sigHandler(int sig) {
    start_ = false;
}

int main(int argc, char **argv) {

#ifdef RS_ROS_FOUND
    ros::init(argc, argv, "receiver_test_demo");
#elif defined(RS_ROS2_FOUND)
    rclcpp::init(argc,argv);
    auto ros2_node = rclcpp::Node::make_shared("rs_sdk_ros2_demo");
#else
    signal(SIGINT, sigHandler);
#endif  // RS_ROS_FOUND

    std::string config_path = std::string(RELEASE_PROJECT_PATH) + "/config";

    RsYamlNode node = rsConfigParser(config_path);

    RsYamlNode communication_node;
    rsYamlSubNode(node, "communication", communication_node);
    perception::Receiver::Ptr receiver_ptr(new perception::Receiver);
    receiver_ptr->init(communication_node);

#ifdef RS_ROS_FOUND
    RsYamlNode perception_node ,rviz_node;
    rsYamlSubNode(node, "perception", perception_node);
    rsYamlSubNode(perception_node, "rviz", rviz_node);
    perception::RvizDisplay::Ptr rviz_display_ptr(new perception::RvizDisplay);
    rviz_display_ptr->init(rviz_node);

    auto func = [rviz_display_ptr](const perception::RsPerceptionMsg::Ptr& msg_ptr) {
        rviz_display_ptr->display(msg_ptr);
    };

    receiver_ptr->registerRcvComm(func);
    RINFO << "include ROS1, init ready!";
    ros::spin();
#elif defined(RS_ROS2_FOUND)
    rclcpp::spin(ros2_node);
    RINFO << "include ROS2, init ready!";
#else
    RINFO << "without ROS! init ready!";
    while (start_) {
        sleep(1);
    }
#endif  // RS_ROS_FOUND
    return 0;
}