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

#include "rviz_display/external/rviz_display.h"
#include "rs_perception/lidar/util/external/rs_auto_align.h"
#include "rs_perception/communication/external/sender.h"
#include "rs_sensor/manager.h"
#include "rs_perception/common/external/rs_util.h"

#ifdef RS_ROS_FOUND
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#endif  // RS_ROS_FOUND

using namespace robosense;
using namespace robosense::perception;

static std::string main_frame_id;
static AutoAlign::Ptr auto_align_ptr;

#ifdef RS_ROS_FOUND
static ros::Publisher pub_plane_fit_cloud;
#else
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
#endif  // RS_ROS_FOUND

int main(int argc, char **argv) {

#ifdef RS_ROS_FOUND
    ros::init(argc, argv, "auto_align_tool");
    ros::NodeHandlePtr node_ptr;
    node_ptr.reset(new ros::NodeHandle);
#endif  // RS_ROS_FOUND

    std::string local_yaml_file = std::string(RELEASE_PROJECT_PATH) + "/test/config.yaml";
    std::string config_path = std::string(RELEASE_PROJECT_PATH) + "/config";

    RsYamlNode node = rsConfigParser(config_path);

    RsYamlNode perception_node;
    rsYamlSubNode(node, "perception", perception_node);
    RsYamlNode auto_align_node;
    rsYamlSubNode(perception_node, "auto_align", auto_align_node);
    auto_align_ptr.reset(new AutoAlign);
    auto_align_ptr->init(auto_align_node);
    auto_align_ptr->setsavepath(std::string(RELEASE_PROJECT_PATH) + "/config/usr_config/calibration.yaml");

    std::shared_ptr<sensor::SensorManager> sensor_ptr_;
    RsYamlNode sensor_config_node;
    rsYamlSubNode(node, "sensor", sensor_config_node);
    main_frame_id = sensor_config_node["main_frame_id"];
    sensor_ptr_.reset(new sensor::SensorManager);
    sensor_ptr_->init(sensor_config_node);

    main_frame_id = auto_align_node["main_frame_id"].as<std::string>();

#ifdef RS_ROS_FOUND
    pub_plane_fit_cloud = node_ptr->advertise<pcl::PointCloud<pcl::PointXYZI> >("aligned_cloud", 1, true);
#endif  // RS_ROS_FOUND

    auto func = [](const lidar::LidarPointCloudMsg &lidar_msg_ptr) {
        if(lidar_msg_ptr.frame_id != main_frame_id){
            return;
        }
        RINFO <<  ": get lidar data of " << lidar_msg_ptr.frame_id
              <<" point num:  "<<lidar_msg_ptr.point_cloud_ptr->size();
        RsPose::Ptr pose_ptr(new RsPose);
        RsPointCloud<RsPoint>::Ptr in_cloud_ptr(new RsPointCloud<RsPoint>);
        in_cloud_ptr->points.resize(lidar_msg_ptr.point_cloud_ptr->size());
        memcpy(in_cloud_ptr->points.data(), lidar_msg_ptr.point_cloud_ptr->points.data(),
               lidar_msg_ptr.point_cloud_ptr->size() * sizeof(RsPoint));
        auto_align_ptr->align(in_cloud_ptr,pose_ptr);

#ifdef RS_ROS_FOUND
        Eigen::Matrix4f trans_mat;
        RsMatrix4f matrix_temp = poseToMat(*pose_ptr);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                trans_mat(i, j) = matrix_temp.val[i][j];
            }
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr ori_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        transRsCloudToPclCloud(in_cloud_ptr, ori_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZI>::Ptr plane_fit_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*ori_cloud_ptr,*plane_fit_cloud_ptr,trans_mat);
        plane_fit_cloud_ptr->header.frame_id = "base_link";
        pub_plane_fit_cloud.publish(plane_fit_cloud_ptr);
#endif  // RS_ROS_FOUND
    };
    sensor_ptr_->regRecvCallback(func);
    sensor_ptr_->start();

    RINFO<<"ready!";

#ifdef RS_ROS_FOUND
    ros::spin();
#else
    while (start_) {
        sleep(1);
    }
#endif  // RS_ROS_FOUND
    return 0;
}
