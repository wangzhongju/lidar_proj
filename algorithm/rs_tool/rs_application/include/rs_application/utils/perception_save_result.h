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

#ifndef RS_APPLICATION_UTILS_PERCEPTION_SAVE_RESULT_H_
#define RS_APPLICATION_UTILS_PERCEPTION_SAVE_RESULT_H_

#include <assert.h>
#include <unistd.h>
#include "rs_perception/common/external/msg/rs_perception_msg.h"
#include "rs_common/external/rs_logger.h"
#include "rs_perception/common/external/rs_config_manager.h"
#include "rs_dependence/rs_dependence_manager.h"
#include "rviz_display/external/common/utils.h"

namespace robosense {

const char _enable[] = "enable";
const char _save_percept_result[] = "save_percept_result";
const char _save_pcd[] = "save_pcd";
const char _save_dir[] = "save_dir";

class PerceptionSaveResult {
public:
    using Ptr = std::shared_ptr<PerceptionSaveResult>;

    // load configures from yaml and init save result method.
    // input: yaml node
    inline void init(const RsYamlNode& config) {
        params.load(config);
        params.log(name());
    }

    // Function used to save perception results.
    // input: robosense perception message struct.
    // output: void. txt file and pcd file will be saved to the specified path.
    inline void save(const perception::RsPerceptionMsg::Ptr& msg_ptr) {
        if (!params.enable_) {
            return;
        }
        double timer_ = getTime();
        const auto& res_ptr = msg_ptr->rs_lidar_result_ptr;
        const auto& timestamp = res_ptr->timestamp;
        // save pcd
#ifdef RS_ROS_FOUND
        if (params.save_pcd_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        const auto& lidar_msg_ptr = res_ptr->scan_ptr;
        perception::transRsCloudToPclCloud(lidar_msg_ptr, tmp_cloud_ptr);
        std::string str = params.save_dir_ + "/" + std::to_string(timestamp) + ".pcd";
        if (pcl::io::savePCDFileBinary(str.c_str(), *tmp_cloud_ptr) == -1) {
            RERROR << name() << ": save pcd error ";
        }
    }
#endif  // RS_ROS_FOUND
        // save perception result
        if (params.save_percept_result_) {
            std::string str = params.save_dir_ + "/" + std::to_string(timestamp) + ".txt";
            std::ofstream outfile;
            outfile.open(str.c_str());
            assert(outfile.is_open());
            outfile << infos(res_ptr);
            outfile.close();
        }
        RDEBUG << name() << ": " << " cost " << (getTime() - timer_) * 1000. << " ms.";
    }

private:
    inline std::string name() {
        return "PerceptionSaveResult";
    }

    struct Params {
        bool enable_ = false;
        bool save_percept_result_ = true;
        bool save_pcd_ = true;
        std::string save_dir_ = "";
        inline void load(const RsYamlNode& config_node) {
            rsYamlRead(config_node, _enable, enable_);
            rsYamlRead(config_node, _save_percept_result, save_percept_result_);
            rsYamlRead(config_node, _save_pcd, save_pcd_);
            rsYamlRead(config_node, _save_dir, save_dir_);
            if (access(save_dir_.c_str(), F_OK) != 0) {
                RDEBUG << "PerceptionSaveResult: can not find save dir: " <<save_dir_;
                enable_ = false;
            }
        }
        inline void log(const std::string& name) {
            RsYamlNode node;
            node[_enable] = enable_;
            node[_save_percept_result] = save_percept_result_;
            node[_save_pcd] = save_pcd_;
            node[_save_dir] = save_dir_;
            RsYamlNode up_node;
            up_node[name] = node;
            std::stringstream ss;
            up_node.serialize(ss);
            ss << std::endl;
            perception::RsConfigManager().append(ss.str());
        }
    } params;

    inline std::string infos(const perception::RsLidarFrameMsg::Ptr &msg_ptr) {
        std::ostringstream os;
        const auto& objects = msg_ptr->objects;
        os << "objects:(object_id),(type),(type_confidence),(size),(center),(direction),(track_id),(vel),(acc)"
            << std::endl;
        for (size_t i = 0; i < objects.size(); ++i) {
            const auto& obj = objects[i];
            os << ":(" << i << "),(" << perception::kObjectType2NameMap.at(obj->core_infos_.type)<< "),(" <<
                obj->core_infos_.type_confidence<< "),(" << obj->core_infos_.size<< "),(" <<
                obj->core_infos_.center << "),(" << obj->core_infos_.direction << "),(" <<
                obj->core_infos_.tracker_id<< "),(" << obj->core_infos_.velocity << "),(" <<
                obj->core_infos_.acceleration << ")" << std::endl;
        }
        return os.str();
    }
};

}  // namespace robosense

#endif  // RS_APPLICATION_UTILS_PERCEPTION_SAVE_RESULT_H_
