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
#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_PROTO_CUSTOM_TRANSFORMER_UTILS_H
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_PROTO_CUSTOM_TRANSFORMER_UTILS_H

#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_PROTO_FOUND

#include "Proto_msg.Percept.pb.h"
#include "rs_perception/custom/common/base_custom_msg.h"
#include "rs_perception/communication/external/common/basic_type.h"

namespace robosense {
namespace perception {
namespace native_sdk_3_1 {

struct st_RoboProtoRecvMessage {
public:
    int device_id;
    double timestamp;
    std::map<ROBO_PROTO_DATA_TYPE, st_RoboMsgRecorder> recorder;
    std::map<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS> status;
    RsPerceptionMsg::Ptr msg;

public:
    st_RoboProtoRecvMessage() {
        device_id = 0;
        timestamp = 0;
    };
public:
    // #ifdef CUSTOM_GE_COMM_FOUND
    //     void updateStatusByConfig(const st_CustomMessageConfig
    //     &fordMessageConfig);
    // #endif // CUSTOM_GE_COMM_FOUND

    // #ifdef CUSTOM_CR_COMM_FOUND
    //     void updateStatusByConfig(const st_CustomMessageConfig
    //     &beiqiMessageConfig);
    // #endif // CUSTOM_CR_COMM_FOUND

    void updateStatusByConfig(const RsProtoCustomMsgParams &translateConfig) {
        status.clear();
        recorder.clear();

        // Message Receive Recorder
        st_RoboMsgRecorder msgRecorder;

        // Object Message Info
        if (translateConfig.send_objects) {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_PROTO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT, msgRecorder));

        // Attention Object Message Info
        if (translateConfig.send_attention_objects) {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_PROTO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT, msgRecorder));

        // Freespace Message Info
        if (translateConfig.send_freespace) {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_PROTO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE, msgRecorder));

        // Lane Message Info
        if (translateConfig.send_lane) {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_PROTO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE, msgRecorder));

        // roadedge Message Info
        if (translateConfig.send_roadedge) {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_PROTO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB, msgRecorder));

        // Axis Lidar Pose Message Info
        if (translateConfig.send_pose) {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_PROTO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE, msgRecorder));

        // point cloud Message Info
        if (translateConfig.send_point_cloud) {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT, ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_LOSS));
        } else {
            status.insert(std::pair<ROBO_PROTO_DATA_TYPE, ROBO_MSG_DATA_RECV_STATUS>(
            ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT,
            ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE));
        }
        recorder.insert(std::pair<ROBO_PROTO_DATA_TYPE, st_RoboMsgRecorder>(
        ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT, msgRecorder));

    }

};
class RsProtoSerializeUtils {
public:
    using Ptr = std::shared_ptr<RsProtoSerializeUtils>;
    using ConstPtr = std::shared_ptr<const RsProtoSerializeUtils>;

public:
    // Object
    static int serialize(const Object::Ptr& object, const int& device_id, bool isSupplement, Proto_msg::Object& proto_object);

    static int deserialize(const Proto_msg::Object& proto_object, Object::Ptr& object, int& device_id);

    // VecObjectPtr
    static int serialize(const VecObjectPtr& objects, const int& device_ids, bool isSupplement, std::vector<Proto_msg::Object>& proto_objects);

    static int deserialize(const std::vector<Proto_msg::Object>& proto_objects, VecObjectPtr& objects, int& device_id);

    // Freespace
    static int serialize(const RsFreeSpace::Ptr& freespaces, const double& timestamp, const int& device_id, Proto_msg::FreeSpaces& proto_freespaces);

    static int deserialize(const Proto_msg::FreeSpaces& proto_freespaces, RsFreeSpace::Ptr& freespaces, double& timestamp, int& device_id);
    
    // Curb
    static int serialize(const std::vector<Roadedge::Ptr>& curbs, const double& timestamp, const int& device_id, Proto_msg::RoadEdges& proto_curbs);

    static int deserialize(const Proto_msg::RoadEdges& proto_curbs, std::vector<Roadedge::Ptr>& curbs, double& timestamp, int& device_id);
    
    // Lane
    static int serialize(const std::vector<Lane::Ptr>& lanes, const double& timestamp, const int& device_id, Proto_msg::Lanes& proto_lanes);

    static int deserialize(const Proto_msg::Lanes& proto_lanes, std::vector<Lane::Ptr>& lanes, double& timestamp, int& device_id);

    // Global Pose
    static int serialize(const RsPose::Ptr& pose, const double& timestamp, const int& device_id, Proto_msg::PoseInfo& proto_pose);

    static int deserialize(const Proto_msg::PoseInfo& proto_pose, RsPose::Ptr& robo_pose, double& timestamp, int& device_id);

    // Point Cloud with sematic indices and object cloud indices
    static int serialize(const RsPointCloudGPT::Ptr& seg_pointCloud, const VecInt& point_label, const double& timestamp, Proto_msg::PointCloud& proto_pointCloud);

    static int deserialize(const Proto_msg::PointCloud& proto_pointCloud, RsPointCloudGPT::Ptr& robo_pointCloud, VecInt& point_label, double& timestamp);

private:

    static void fromEigenToProto(const RsVector3f &eig, Proto_msg::Point3f *point) {
        point->add_data(eig.x);
        point->add_data(eig.y);
        point->add_data(eig.z);
    }

    static void fromProtoToEigen(const Proto_msg::Point3f &point, RsVector3f &eig) {
        eig.x = point.data(0);
        eig.y = point.data(1);
        eig.z = point.data(2);
    }

    static void fromEigenToProto(const RsVector2f& eig, Proto_msg::Point2f *point) {
        point->add_data(eig.x);
        point->add_data(eig.y);
    }

    static void fromProtoToEigen(const Proto_msg::Point2f& point, RsVector2f& eig) {
        eig.x = point.data(0);
        eig.y = point.data(1);
    }

    static void fromEigenToProto(const Curve& robo_curve, Proto_msg::Curve *proto_curve) {
        proto_curve->set_x_start(robo_curve.x_start);
        proto_curve->set_x_end(robo_curve.x_end);
        proto_curve->set_a(robo_curve.a);
        proto_curve->set_b(robo_curve.b);
        proto_curve->set_c(robo_curve.c);
        proto_curve->set_d(robo_curve.d);
    }

    static void fromProtoToEigen(const Proto_msg::Curve& proto_curve, Curve& robo_curve) {
        robo_curve.x_start = proto_curve.x_start();
        robo_curve.x_end = proto_curve.x_end();
        robo_curve.a = proto_curve.a();
        robo_curve.b = proto_curve.b();
        robo_curve.c = proto_curve.c();
        robo_curve.d = proto_curve.d();
    }
};



class RSProtoDeserializeUtil {
public:
    using Ptr = std::shared_ptr<RSProtoDeserializeUtil>;
    using ConstPtr = std::shared_ptr<const RSProtoDeserializeUtil>;

public:
    bool isEmptyMsgType(const ROBO_PROTO_DATA_TYPE msgType) {
        if (msgType == ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT_EMPTY
        || msgType == ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT_EMPTY
        || msgType == ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE_EMPTY
        || msgType == ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE_EMPTY
        || msgType == ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB_EMPTY
        || msgType == ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT_EMPTY
        || msgType == ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_AXISSTATUS_EMPTY
        || msgType == ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE_EMPTY
        || msgType == ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_AXISLIDAR_POSE_EMPTY) {
            return true;
        }

        return false;
    }

    bool checkDeserializeStatus(const st_RoboProtoRecvMessage &recvMsg, uint16_t msgType) {
        // 发的不是空消息，但是不解析，返回true
        // 发的不是空消息，且要解析，返回false
        native_sdk_3_1::ROBO_PROTO_DATA_TYPE msg_type = static_cast<native_sdk_3_1::ROBO_PROTO_DATA_TYPE>(msgType);
        const auto& status_map = recvMsg.status;
        if (status_map.at(msg_type) == native_sdk_3_1::ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_NO_TRANSLATE) {
            return true;
        }
        else {
            return false;
        }
    }

    bool checkMessageComplete(const RsProtoCustomMsgParams::Ptr& custom_params_, st_RoboProtoRecvMessage &recvMsg) {
        if (custom_params_->send_objects &&
        recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT] != native_sdk_3_1::ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_OBJECT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "1 Incomplete" << std::endl;
            return false;
        }

        if (custom_params_->send_attention_objects &&
        recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_ATT_OBJECT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "2 Incomplete" << std::endl;
            return false;
        }

        if (custom_params_->send_freespace &&
        recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_FREESPACE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "3 Incomplete" << std::endl;
            return false;
        }

        if (custom_params_->send_lane &&
        recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_LANE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "4 Incomplete" << std::endl;
            return false;
        }

        if (custom_params_->send_roadedge &&
        recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_CURB] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "5 Incomplete" << std::endl;
            return false;
        }

        if (custom_params_->send_point_cloud &&
        recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_POINT] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "6 Incomplete" << std::endl;
            return false;
        }

        if (custom_params_->send_pose &&
        recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_EMPTY
        && recvMsg.status[ROBO_PROTO_DATA_TYPE::ROBO_PROTO_DATA_GLOBALCAR_POSE] != ROBO_MSG_DATA_RECV_STATUS::ROBO_MSG_DATA_SUCCESS) {
            //            std::cout << "7 Incomplete" << std::endl;
            return false;
        }

        return true;
    }
};

}  // namespace native_sdk_3_1
}  // namespace perception
}  // namespace robosense

#endif  // RS_PROTO_FOUND
#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_PROTO_CUSTOM_TRANSFORMER_UTILS_H
