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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_WEBSOCKET_WEBSOCKET_CUSTOM_TRANSFORMER_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_WEBSOCKET_WEBSOCKET_CUSTOM_TRANSFORMER_H_
#include "rs_perception/custom/robosense_websocket/websocket_custom_transformer_util.h"

#ifdef ROBOSENSE_WEBSOCKET_FOUND
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rs_perception/custom/common/base_custom_params.h"
#include "rs_perception/communication/external/common/basic_type.h"
#include "rs_perception/custom/robosense_websocket/websocket_color_mapper.h"


// In order to use JSON::FastWriter Without Warning
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(disable : 4996)
#endif

namespace robosense {
namespace perception {

class RsWebsocketSerialize {
public:
    typedef std::shared_ptr<RsWebsocketSerialize> Ptr;
    typedef std::shared_ptr<const RsWebsocketSerialize> ConstPtr;

private:
    class RsXvizPoint3D {
    public:
        double x;
        double y;
        double z;

        std::string json_array;

        RsXvizPoint3D() {
            x = 0;
            y = 0;
            z = 0;

            json_array = "";
        }

        void updateJsonArray() {
            json_array.clear();
            json_array = "[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]";
        }
    };

public:
    RsWebsocketSerialize(const RsCustomWebsocketMsgParams &commConfig);

    ~RsWebsocketSerialize();

    int serialize(const robosense::perception::RsPerceptionMsg::Ptr &msg, const native_sdk_3_1::ROBO_MSG_TYPE msgType,
                  native_sdk_3_1::RSSerializeBuffer &en_msg);

    // Initial
    int init(const double timestampGapS);

private:
    // Make Robosense P5 Metadata
    xvizGLTF::Ptr makeRobosenseMetaData();

    // Make Robosense Self Define Data
    xvizGLTF::Ptr makeRobosenseMessageData(const robosense::perception::RsPerceptionMsg::Ptr &msg);

    // Serialize GLTF Frame to Buffer
    int gltfSerialize(const xvizGLTF::Ptr &gltfPtr, std::vector<char> &serializeBuffer);

    int loadFillMap();

    std::string fromTypeIDToType(const int objTypeId);

    int makeSerializeObjectCorners(const std::vector<robosense::perception::Object::Ptr> &objects,
                                   std::vector<std::vector<RsXvizPoint3D>> &objectsCorners);

    int makeSerializeObjectPolygons(const std::vector<robosense::perception::Object::Ptr> &objects,
                                    std::vector<std::vector<RsXvizPoint3D>> &objectsPolygons);

    int makeSerializeAttentionObjectPolygons(const std::vector<robosense::perception::Object::Ptr> &objects,
                                             std::vector<std::vector<RsXvizPoint3D>> &objectsPolygons);

    // "vertices":[[x_1, y_1, z_1], ... , [x_n, y_n, z_n]]
    int makeSerializeCubeBox(const std::vector<RsXvizPoint3D> &cornerPolygons, std::string &vertices);

    // "vertices":[[x_1,y_1,z_1],...,[x_n,y_n,z_n]]
    int makeSerializeAttentionCubeBox(const std::vector<RsXvizPoint3D> &cornerPolygons, std::string &vertices);

    // "vertices":[[x_1, y_1, z_1],...,[x_n, y_n, z_n]]
    int makeSerializeGridBox(const std::vector<RsXvizPoint3D> &cornerPolygons, std::string &vertices);

    // "vertices":[[x_1, y_1, z_1],...,[x_n, y_n, z_n]]
    int makeSerializeAttentionGridBox(const std::vector<RsXvizPoint3D> &cornerPolygons, std::string &vertices);

    // lane stream
    int makeSerializeLanes(const std::vector<robosense::perception::Lane::Ptr> &lanes, const std::string &stream_id,
                           const std::string &type, std::string &vertices);

    // curb stream
    int makeSerializeCurbs(const std::vector<robosense::perception::Roadedge::Ptr> &curbs, const std::string &stream_id,
                           const std::string &type, std::string &vertices);

    // ROI Stream
    int makeSerializeRoi(const RsXvizRoiInfo &roiInfo, const std::string &stream_id, const std::string &type,
                         const std::string &object_id, std::string &vertices);

    // freespace stream
    int makeSerializeFreespace(const std::vector<RsVector3f> &freespaces, const std::string &stream_id,
                               const std::string &type, std::string &vertices);

    // vector<""center"":[x, y, z]>
    // int makeSerializeTrajectory(const boost::circular_buffer<Eigen::Vector3f>
    // &trajectory, std::vector<std::string> &vertices);
    int makeSerializeTrajectory(const std::deque<RsVector3f> &trajectory, std::vector<std::string> &vertices);

    int makeSerializeBufferViews(const std::vector<std::pair<int, int>> &bufferInfos, std::string &bufferViews);

    // Point Accessors: accessorInfos[0]: colors, accessorInfos[1]: points
    int makeSerializeAccessors(const std::vector<int> &accessorInfos, std::string &accessors);

    int makeSerializeBuffers(const int byteLength, std::string &buffers);

    int makeSerializeImageBuffers(const int bufferViewOffset, const int imageBufferViewCnt,const int imageWidth,
                                  const int imageHeight, std::string &images);

    int makeSerializeMesh(std::string &mesh);

    // Make Pose Stream
    int makeSerializePoseStream(const std::string &stream_id, const double timestamp,
                                const double map_ori_x, const double map_ori_y, const double map_ori_z,
                                const double pos_x, const double pos_y, const double pos_z,
                                const double ori_x, const double ori_y, const double ori_z,
                                std::string &pose);

    // Make Image Streams
    int makeSerializeImageStreams(const std::vector<std::string> &streamIds, const int imageWidth,
                                  const int imageHeight, std::string &imageStreams);

    // Make Single Image Stream
    int makeSerializeImageStream(const std::string &streamId, const int imageBufferIdx, const int imageWidth,
                                 const int imageHeight, std::string &imageStream);

    // Make Point Stream
    int makeSerializePointStream(const std::string &streamId, std::string &pointStream,
                                 const std::string &object_id = "robo_lidar_points", const int colorAccessor = 0,
                                 const int pointAccessor = 1);

    // Make Map Stream
    int makeSerializeMapStream(const std::string &streamId, std::string &mapStream,
                               const std::string &object_id = "robo_map_points", const int colorAccessor = 0,
                               const int pointAccessor = 1);

    template <typename T>
    std::string num2str(const T num, int precision) {
        std::stringstream ss;
        ss.setf(std::ios::fixed, std::ios::floatfield);
        ss.precision(precision);
        std::string st;
        ss << num;
        ss >> st;

        return st;
    }

    xvizGLTF::Ptr _metadata;
    xvizGLTF::Ptr _message;

    // Robosense Map Data
    std::vector<int> _mapPointCntBuffers;
    std::vector<std::vector<char>> _mapPointBuffers;
    std::vector<std::vector<char>> _mapColorBuffers;

    std::vector<char> _attentionPolygonBuffer;
    std::vector<char> _attentionPolyhedralBuffer;
    std::vector<char> _gridCircleBuffer;
    std::vector<char> _pointsBuffer;
    std::vector<char> _objectBoxesBuffer;
    std::vector<char> _objectCubesBuffer;
    std::vector<char> _objectPolygonBuffer;
    std::vector<char> _objectPolyhedralBuffer;
    std::vector<char> _trackingPointBuffer;
    std::vector<char> _trajectoryBuffer;
    std::vector<char> _velocityDirBuffer;
    std::vector<char> _boxInfoBuffer;
    std::vector<char> _labelInfoBuffer;
    std::vector<char> _trackInfoBuffer;
    std::vector<char> _gpsInfoBuffer;

    // Stream(s) or GLTF Elements
    std::string _accessorsBuffer;
    std::string _bufferViewsBuffers;
    std::string _buffersBuffers;
    std::string _imagesBuffers;
    std::string _imageStreamBuffers;
    std::string _meshesBuffers;
    std::string _poseStreamBuffers;
    std::string _gridCircleStreamBuffers;
    std::string _lidarPointStreamBuffers;
    std::vector<std::string> _mapPointStreamBuffers;
    std::string _freespaceBuffer;
    std::string _lanesBuffer;
    std::string _curbsBuffer;
    std::vector<std::string> _roisBuffer;

    int _objectbox_offset;
    int _objectcube_offset;
    int _objectpolygon_offset;
    int _objectpolyhedral_offset;
    int _trackingpoint_offset;
    int _trajectory_offset;
    int _velocitydir_offset;
    int _boxinfo_offset;
    int _labelinfo_offset;
    int _trackinfo_offset;
    int _gpsinfo_offset;

    unsigned char _pointCloudColors[4];
    unsigned char _mapColors[4];

    int _attentionpolyhedral_offset;
    int _attentionpolygon_offset;

    double _logStartTime;
    double _logEndTime;
    RsCustomWebsocketMsgParams _commConfig;

    int _totalFrameCnt;
    const int _defaultBufferCnt = 2 * 1024 * 1024;  // 2 Mbyte
    const int _defaultImageWidth = 397;
    const int _defaultImageHeight = 120;
};

}  // namespace perception
}  // namespace robosense

#endif  // ROBOSENSE_WEBSOCKET_FOUND

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_WEBSOCKET_WEBSOCKET_CUSTOM_TRANSFORMER_H_
