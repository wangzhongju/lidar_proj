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

#include "rs_perception/custom/robosense_websocket/websocket_custom_transformer.h"

#ifdef ROBOSENSE_WEBSOCKET_FOUND

namespace robosense {
namespace perception {

RsWebsocketSerialize::RsWebsocketSerialize( const RsCustomWebsocketMsgParams &commConfig)
: _commConfig(commConfig) {
    // TODO...
}

RsWebsocketSerialize::~RsWebsocketSerialize() {
    // TODO...
}

int RsWebsocketSerialize::serialize(
    const robosense::perception::RsPerceptionMsg::Ptr &msg,
    const native_sdk_3_1::ROBO_MSG_TYPE msgType,
    native_sdk_3_1::RSSerializeBuffer &en_msg) {
    if ((msg == nullptr && msgType != native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_WEBSOCEKT_METADATA)) {
        return -1;
    }

    int gltfSize = 0;
    if (msgType == native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_WEBSOCEKT_METADATA) {
        makeRobosenseMetaData();
        gltfSize = gltfSerialize(_metadata, en_msg.buffers);
    }
    else if (msgType == native_sdk_3_1::ROBO_MSG_TYPE::ROBO_MSG_WEBSOCKET_MESSAGE) {
        makeRobosenseMessageData(msg);

        gltfSize = gltfSerialize(_message, en_msg.buffers);
    }

    en_msg.lengths.push_back(gltfSize);
    en_msg.offsets.push_back(0);
    en_msg.types.push_back(msgType);
    en_msg.msgCntMap.insert(std::pair<native_sdk_3_1::ROBO_MSG_TYPE, int>(msgType, 1));

    return 0;
}

// Initial
int RsWebsocketSerialize::init(const double timestampGapS) {
    try {
        _metadata.reset(new xvizGLTF);
        _message.reset(new xvizGLTF);
    }
    catch (std::exception &e) {
        std::cout << "Initial XVIZ Make Failed: " << e.what() << std::endl;
        return -1;
    }

    int ret;
    if (_commConfig.enablesConfig.mapEnables.maps_enable) {
        ret = loadFillMap();

        if (ret != 0) {
            std::cout << "Load Fill Map For Websocket Failed" << std::endl;
            return -4;
        }
    }

    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>
    currentTime = std::chrono::time_point_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now());
    _logStartTime = currentTime.time_since_epoch().count();

    _logStartTime /= 1000.0f;

    if (timestampGapS < 0.0f) {
        _logEndTime = _logStartTime + 24 * 60 * 60 * 365;
    }
    else {
        _logEndTime = _logStartTime + timestampGapS;
    }

    // ==================================================
    // =============== Initial Only Once ================
    // ==================================================
    // girdcirlce stream without json
    {
        _gridCircleBuffer.resize(_defaultBufferCnt);
        int gridcircle_offset = 0;

        const static std::string gridcircle_streamid = "/grids/grid_circles";
        const static std::string gridcircle_type = "circles";

        const static std::string gridcircle_start =
        "\"" + gridcircle_streamid + "\":{\"" + gridcircle_type + "\":[";
        memcpy(_gridCircleBuffer.data() + gridcircle_offset,
        gridcircle_start.data(), gridcircle_start.size());
        gridcircle_offset += gridcircle_start.size();

        const static std::string girdcircle_end = "]}";

        int gridCircleCnt = _commConfig.lidarGridConfig.grid_circle_ranges.size();
        int gridCircleCnt1 = gridCircleCnt - 1;

        std::string json_array;
        for (int iterCircle = 0; iterCircle < gridCircleCnt; ++iterCircle) {
            std::string base0 = std::string("{\"base\":{\"object_id\":\"") +
            std::to_string(iterCircle) + "\"},";

            if (iterCircle != gridCircleCnt1) {
                json_array = base0 + "\"center\":[0.0, 0.0, 0.0],\"radius\":" +
                std::to_string(_commConfig.lidarGridConfig.grid_circle_ranges[iterCircle]) + "},";
                }
            else {
            json_array = base0 + "\"center\":[0.0, 0.0, 0.0],\"radius\":" +
            std::to_string(_commConfig.lidarGridConfig.grid_circle_ranges[iterCircle]) + "}";
            }

            memcpy(_gridCircleBuffer.data() + gridcircle_offset, json_array.data(), json_array.size());
            gridcircle_offset += json_array.size();
        }
        memcpy(_gridCircleBuffer.data() + gridcircle_offset, girdcircle_end.data(), girdcircle_end.size());
        gridcircle_offset += girdcircle_end.size();

        _gridCircleBuffer.resize(gridcircle_offset);
    }

    // Make Lidar Point Stream Without Json
    int pointAccessorOffset = 0;
    if (_commConfig.enablesConfig.pointCloudEnables.pointcloud_enable) {
        const static std::string lidarpoint_streamid = "/lidar/points";
        const static std::string lidarpoint_objectid = "robo_lidar_points";
        int accessorOffset = pointAccessorOffset;
        int accessorOffset1 = pointAccessorOffset + 1;
        makeSerializePointStream(lidarpoint_streamid, _lidarPointStreamBuffers,
                                 lidarpoint_objectid, accessorOffset, accessorOffset1);
        pointAccessorOffset += 2;
    }

    // Make Map Streams Without Json
    if (_commConfig.enablesConfig.mapEnables.maps_enable) {
        size_t fillMapsCnt = _commConfig.fillMapsConfig.maps.size();
        _mapPointStreamBuffers.clear();
        if (fillMapsCnt > 0) {
            _mapPointStreamBuffers.resize(_commConfig.fillMapsConfig.maps.size());
            for (size_t i = 0; i < _commConfig.fillMapsConfig.maps.size(); ++i) {
                std::string map_streamid = "/map/" + std::to_string(i) + "/points";
                std::string map_objectid = "robo_map" + std::to_string(i) + "_points";
                int accessorOffset = pointAccessorOffset;
                int accessorOffset1 = pointAccessorOffset + 1;
                makeSerializeMapStream(map_streamid, _mapPointStreamBuffers[i],
                                       map_objectid, accessorOffset, accessorOffset1);
                pointAccessorOffset += 2;
            }
        }
    }

    // Make Roi Streams Without Json
    if (_commConfig.enablesConfig.roiEnables.rois_enable) {
        // std::cout << "run here AAAAA" << std::endl;
        size_t fillRoisCnt = _commConfig.fillRoisConfig.roiInfos.size();
        _roisBuffer.resize(fillRoisCnt);

        for (size_t i = 0; i < fillRoisCnt; ++i) {
            const std::string stream_id = "/roi/" + std::to_string(i) + "/polygons";
            const std::string type = "polygons";
            const std::string object_id = "roi" + std::to_string(i);
            makeSerializeRoi(_commConfig.fillRoisConfig.roiInfos[i], stream_id,
                             type, object_id, _roisBuffer[i]);
        }
    }

    // Make GLTF Element: meshes without Json
    makeSerializeMesh(_meshesBuffers);

    _objectBoxesBuffer.resize(_defaultBufferCnt);
    _objectCubesBuffer.resize(_defaultBufferCnt);
    _objectPolygonBuffer.resize(_defaultBufferCnt);
    _objectPolyhedralBuffer.resize(_defaultBufferCnt);
    _trackingPointBuffer.resize(_defaultBufferCnt);
    _trajectoryBuffer.resize(_defaultBufferCnt);
    _velocityDirBuffer.resize(_defaultBufferCnt);
    _boxInfoBuffer.resize(_defaultBufferCnt);
    _labelInfoBuffer.resize(_defaultBufferCnt);
    _trackInfoBuffer.resize(_defaultBufferCnt);
    _gpsInfoBuffer.resize(_defaultBufferCnt);

    _attentionPolyhedralBuffer.resize(_defaultBufferCnt);
    _attentionPolygonBuffer.resize(_defaultBufferCnt);

    // Setting Default Mapper Color
    memcpy(_pointCloudColors, _commConfig.colorMapConfig.defaultColor.data(),sizeof(_pointCloudColors));
    memcpy(_mapColors, _commConfig.mapColorMapConfig.defaultColor.data(), sizeof(_mapColors));

    return 0;
}

// Make Robosense P5 Metadata
xvizGLTF::Ptr RsWebsocketSerialize::makeRobosenseMetaData() {
    Json::Value kittiMetadata(Json::ValueType::objectValue);

    Json::Value buffers(Json::ValueType::arrayValue);
    Json::Value firstBuffers(Json::ValueType::objectValue);
    firstBuffers["byteLength"] = 0;
    buffers.append(firstBuffers);

    kittiMetadata["buffers"] = buffers;

    Json::Value bufferViews(Json::ValueType::arrayValue);
    kittiMetadata["bufferViews"] = bufferViews;

    Json::Value accessors(Json::ValueType::arrayValue);
    kittiMetadata["accessors"] = accessors;

    Json::Value meshes(Json::ValueType::arrayValue);
    kittiMetadata["meshes"] = meshes;

    Json::Value xviz(Json::ValueType::objectValue);

    xviz["type"] = "xviz/metadata";

    xvizMetaBuilder metaBuilder;

    // Stream
    metaBuilder.stream("/vehicle_pose");
    metaBuilder.category("pose");

    // Object transform
    Json::Value transform(Json::ValueType::arrayValue);
    int maxtrixCnt = _commConfig.transformConfig.matrix.size();
    for (int i = 0; i < maxtrixCnt; ++i) {
        transform.append(_commConfig.transformConfig.matrix[i]);
    }

    // Stream: polyhedral attention
    if (_commConfig.enablesConfig.attentionEnables.attention_enable_polyhedral) {
        metaBuilder.stream("/attentions/attention_polyhedral");
        metaBuilder.category("primitive");
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        Json::Value objectStyle(Json::ValueType::objectValue);
        RsXvizColorConfig objectConfig = _commConfig.attentionObjectConfigs;
        std::string color = objectConfig.fromVecToColorString();
        objectStyle["extruded"] = true;
        objectStyle["fill_color"] = color;
        objectStyle["stroke_color"] = color.substr(0, color.size() - 2);
        metaBuilder.streamStyle(objectStyle);
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polygon");
    }

    // Stream: attention_polygon
    if (_commConfig.enablesConfig.attentionEnables.attention_enable_polygon) {
        metaBuilder.stream("/attentions/attention_polygon");
        metaBuilder.category("primitive");
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        Json::Value objectStyle(Json::ValueType::objectValue);
        // attention polygon
        RsXvizColorConfig objectConfig = _commConfig.attentionObjectConfigs;
        std::string color = objectConfig.fromVecToColorString();
        objectStyle["extruded"] = true;
        objectStyle["fill_color"] = color;
        objectStyle["stroke_color"] = color.substr(0, color.size() - 2);

        metaBuilder.streamStyle(objectStyle);

        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polyline");
    }

    // Stream: object cubes
    if (_commConfig.enablesConfig.objectEnables.object_enable_cube) {
        metaBuilder.stream("/objects/object_cubes");
        metaBuilder.category("primitive");
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        Json::Value objectStyle(Json::ValueType::objectValue);
        objectStyle["extruded"] = true;
        objectStyle["fill_color"] = "#FF000080";
        objectStyle["stroke_color"] = "#FF0000";
        metaBuilder.streamStyle(objectStyle);
        int objectConfigsCnt = _commConfig.objectConfigs.size();
        for (int i = 0; i < objectConfigsCnt; ++i) {
            RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
            Json::Value classStyle(Json::ValueType::objectValue);
            std::string color = objectConfig.color.fromVecToColorString();
            classStyle["fill_color"] = color;
            classStyle["stroke_color"] = color.substr(0, color.size() - 2);
            metaBuilder.classStyle(objectConfig.type, classStyle);
        }
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polygon");
    }

    // Stream: boxes
    if (_commConfig.enablesConfig.objectEnables.object_enable_box) {
        metaBuilder.stream("/objects/object_boxes");
        metaBuilder.category("primitive");
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        Json::Value objectStyle(Json::ValueType::objectValue);
        objectStyle["extruded"] = true;
        objectStyle["fill_color"] = "#FF000080";
        objectStyle["stroke_color"] = "#FF0000";

        metaBuilder.streamStyle(objectStyle);
        int objectConfigsCnt = _commConfig.objectConfigs.size();
        for (int i = 0; i < objectConfigsCnt; ++i) {
            RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
            Json::Value classStyle(Json::ValueType::objectValue);
            std::string color = objectConfig.color.fromVecToColorString();
            classStyle["fill_color"] = color;
            classStyle["stroke_color"] = color.substr(0, color.size() - 2);
            metaBuilder.classStyle(objectConfig.type, classStyle);
        }
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polyline");
    }

    // Stream: object polyhedral
    if (_commConfig.enablesConfig.objectEnables.object_enable_polyhedral) {
        metaBuilder.stream("/objects/object_polyhedral");
        metaBuilder.category("primitive");
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        Json::Value objectStyle(Json::ValueType::objectValue);
        objectStyle["extruded"] = true;
        objectStyle["fill_color"] = "#00000080";
        metaBuilder.streamStyle(objectStyle);

        int objectConfigsCnt = _commConfig.objectConfigs.size();
        for (int i = 0; i < objectConfigsCnt; ++i) {
            RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
            Json::Value classStyle(Json::ValueType::objectValue);
            std::string color = objectConfig.color.fromVecToColorString();
            classStyle["fill_color"] = color;
            classStyle["stroke_color"] = color.substr(0, color.size() - 2);
            metaBuilder.classStyle(objectConfig.type, classStyle);
        }

        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polygon");
    }

    // Stream: object polygon
    if (_commConfig.enablesConfig.objectEnables.object_enable_polygon) {
        metaBuilder.stream("/objects/object_polygon");
        metaBuilder.category("primitive");
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        Json::Value objectStyle(Json::ValueType::objectValue);
        objectStyle["extruded"] = true;
        objectStyle["fill_color"] = "#FF000080";
        objectStyle["stroke_color"] = "#FF0000";
        metaBuilder.streamStyle(objectStyle);

        int objectConfigsCnt = _commConfig.objectConfigs.size();
        for (int i = 0; i < objectConfigsCnt; ++i) {
            RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
            Json::Value classStyle(Json::ValueType::objectValue);
            std::string color = objectConfig.color.fromVecToColorString();
            classStyle["fill_color"] = color;
            classStyle["stroke_color"] = color.substr(0, color.size() - 2);
            metaBuilder.classStyle(objectConfig.type, classStyle);
        }
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polyline");
    }

    // Stream: tracking_point
    if (_commConfig.enablesConfig.objectEnables.object_enable_trackingpoint) {
        metaBuilder.stream("/objects/tracking_point");
        metaBuilder.category("primitive");

        // Attach To Object
        if (_commConfig.trackPointConfigs.attach_object) {
            Json::Value trackletsStyle(Json::ValueType::objectValue);
            trackletsStyle["radius"] = 0.15;
            trackletsStyle["stroke_width"] = 0;
            trackletsStyle["fill_color"] = "#FF000080";
            trackletsStyle["stroke_color"] = "#FF000000";
            metaBuilder.streamStyle(trackletsStyle);

            int objectConfigsCnt = _commConfig.objectConfigs.size();
            for (int i = 0; i < objectConfigsCnt; ++i) {
                RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
                Json::Value classStyle(Json::ValueType::objectValue);
                std::string color = objectConfig.color.fromVecToColorString();
                classStyle["fill_color"] = color;
                classStyle["stroke_color"] = color.substr(0, color.size() - 2);
                metaBuilder.classStyle(objectConfig.type, classStyle);
            }
        }
        else {
            Json::Value trackletsStyle(Json::ValueType::objectValue);
            RsXvizTrackPointConfig trackPointConfig = _commConfig.trackPointConfigs;
            std::string color = trackPointConfig.color.fromVecToColorString();
            trackletsStyle["radius"] = 0.15;
            trackletsStyle["stroke_width"] = 0;
            trackletsStyle["fill_color"] = color;
            trackletsStyle["stroke_color"] = color.substr(0, color.size() - 2);

            metaBuilder.streamStyle(trackletsStyle);
        }
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("circle");
    }

    // Stream: velocity_dir
    if (_commConfig.enablesConfig.objectEnables.object_enable_velocitydir) {
        metaBuilder.stream("/objects/velocity_dir");
        metaBuilder.category("primitive");
        Json::Value velocityDirStyle(Json::ValueType::objectValue);
        RsXvizTrackPointConfig velocityDirConfig = _commConfig.velocityDirConfig;
        std::string color = velocityDirConfig.color.fromVecToColorString();
        velocityDirStyle["fill_color"] = color;
        velocityDirStyle["stroke_color"] = color.substr(0, color.size() - 2);
        metaBuilder.streamStyle(velocityDirStyle);

        // Attach To Object
        if (_commConfig.velocityDirConfig.attach_object) {
            int objectConfigsCnt = _commConfig.objectConfigs.size();
            for (int i = 0; i < objectConfigsCnt; ++i) {
                RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
                Json::Value classStyle(Json::ValueType::objectValue);
                std::string color = objectConfig.color.fromVecToColorString();
                classStyle["fill_color"] = color;
                classStyle["stroke_color"] = color.substr(0, color.size() - 2);
                metaBuilder.classStyle(objectConfig.type, classStyle);
            }
        }
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polyline");
    }

    // Stream: trajectory
    if (_commConfig.enablesConfig.objectEnables.object_enable_trajectory) {
        metaBuilder.stream("/objects/trajectory");
        metaBuilder.category("primitive");
        // Attach To Object
        if (_commConfig.trajectoryConfigs.attach_object) {
            Json::Value trackletsStyle(Json::ValueType::objectValue);
            trackletsStyle["radius"] = 0.15;
            trackletsStyle["fill_color"] = "#FF000080";
            trackletsStyle["stroke_color"] = "#FF0000";
            metaBuilder.streamStyle(trackletsStyle);

            int objectConfigsCnt = _commConfig.objectConfigs.size();
            for (int i = 0; i < objectConfigsCnt; ++i) {
                RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
                Json::Value classStyle(Json::ValueType::objectValue);
                std::string color = objectConfig.color.fromVecToColorString();
                classStyle["fill_color"] = color;
                classStyle["stroke_color"] = color.substr(0, color.size() - 2);
                metaBuilder.classStyle(objectConfig.type, classStyle);
            }
        }
        else {
            Json::Value trackletsStyle(Json::ValueType::objectValue);
            trackletsStyle["radius"] = 0.15;
            RsXvizTrajectorConfig trajectoryConfig = _commConfig.trajectoryConfigs;
            std::string color = trajectoryConfig.color.fromVecToColorString();
            trackletsStyle["fill_color"] = color;
            trackletsStyle["stroke_color"] = color.substr(0, color.size() - 2);
            metaBuilder.streamStyle(trackletsStyle);
        }
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("circle");
    }

    // Stream: box_info
    if (_commConfig.enablesConfig.objectEnables.object_enable_boxinfo) {
        metaBuilder.stream("/texts/box_info");
        metaBuilder.category("primitive");

        RsXvizBoxTextConfig boxTextConfig = _commConfig.boxTextConfigs;
    // Attach To Object
        if (_commConfig.boxTextConfigs.attach_object) {
            Json::Value boxInfoStyle(Json::ValueType::objectValue);
            boxInfoStyle["text_size"] = boxTextConfig.text_size;
            boxInfoStyle["fill_color"] = "#FFFFFFFF";
            metaBuilder.streamStyle(boxInfoStyle);
            int objectConfigsCnt = _commConfig.objectConfigs.size();
        for (int i = 0; i < objectConfigsCnt; ++i) {
            RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
            Json::Value classStyle(Json::ValueType::objectValue);
            std::string color = objectConfig.color.fromVecToColorString();
            classStyle["fill_color"] = color;
            classStyle["stroke_color"] = color.substr(0, color.size() - 2);
            metaBuilder.classStyle(objectConfig.type, classStyle);
        }
        }
        else {
            Json::Value boxInfoStyle(Json::ValueType::objectValue);
            boxInfoStyle["text_size"] = boxTextConfig.text_size;
            std::string color = boxTextConfig.color.fromVecToColorString();
            boxInfoStyle["fill_color"] = color;
            boxInfoStyle["stroke_color"] = color.substr(0, color.size() - 2);
            metaBuilder.streamStyle(boxInfoStyle);
        }

        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("text");
    }

    // Stream: label_info
    if (_commConfig.enablesConfig.objectEnables.object_enable_labelinfo) {
        metaBuilder.stream("/texts/label_info");
        metaBuilder.category("primitive");

        RsXvizlabelTextConfig labelTextConfig = _commConfig.labelTextConfigs;
        // Attach To Object
        if (_commConfig.labelTextConfigs.attach_object) {
            Json::Value labelInfoStyle(Json::ValueType::objectValue);
            labelInfoStyle["text_size"] = labelTextConfig.text_size;
            labelInfoStyle["fill_color"] = "#FFFFFFFF";
            metaBuilder.streamStyle(labelInfoStyle);

            int objectConfigsCnt = _commConfig.objectConfigs.size();
            for (int i = 0; i < objectConfigsCnt; ++i) {
                RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
                Json::Value classStyle(Json::ValueType::objectValue);
                std::string color = objectConfig.color.fromVecToColorString();
                classStyle["fill_color"] = color;
                classStyle["stroke_color"] = color.substr(0, color.size() - 2);
                metaBuilder.classStyle(objectConfig.type, classStyle);
            }
        }
        else {
            Json::Value labelInfoStyle(Json::ValueType::objectValue);
            labelInfoStyle["text_size"] = labelTextConfig.text_size;
            std::string color = labelTextConfig.color.fromVecToColorString();
            labelInfoStyle["fill_color"] = color;
            labelInfoStyle["stroke_color"] = color.substr(0, color.size() - 2);
            metaBuilder.streamStyle(labelInfoStyle);
        }
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("text");
    }

    // Stream: track_info
    if (_commConfig.enablesConfig.objectEnables.object_enable_trackinfo) {
        metaBuilder.stream("/texts/track_info");
        metaBuilder.category("primitive");

        RsXvizTrackTextConfig trackTextConfig = _commConfig.trackTextConfigs;

        // Attach To Object
        if (_commConfig.trackTextConfigs.attach_object) {
            Json::Value trackInfoStyle(Json::ValueType::objectValue);
            trackInfoStyle["text_size"] = trackTextConfig.text_size;
            trackInfoStyle["fill_color"] = "#FFFFFFFF";
            metaBuilder.streamStyle(trackInfoStyle);

        int objectConfigsCnt = _commConfig.objectConfigs.size();
            for (int i = 0; i < objectConfigsCnt; ++i) {
                RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
                Json::Value classStyle(Json::ValueType::objectValue);
                std::string color = objectConfig.color.fromVecToColorString();
                classStyle["fill_color"] = color;
                classStyle["stroke_color"] = color.substr(0, color.size() - 2);
                metaBuilder.classStyle(objectConfig.type, classStyle);
            }
        }
        else {
            Json::Value trackInfoStyle(Json::ValueType::objectValue);
            std::string color = trackTextConfig.color.fromVecToColorString();
            trackInfoStyle["text_size"] = trackTextConfig.text_size;
            trackInfoStyle["fill_color"] = color;
            trackInfoStyle["stroke_color"] = color.substr(0, color.size() - 2);

            metaBuilder.streamStyle(trackInfoStyle);
        }

        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("text");
    }

    // Stream: gps_info
    if (_commConfig.enablesConfig.objectEnables.object_enable_gpsinfo) {
        metaBuilder.stream("/texts/gps_info");
        metaBuilder.category("primitive");

        RsXvizGpsTextConfig gpsTextConfig = _commConfig.gpsTextConfigs;
        // Attach To Object
        if (_commConfig.gpsTextConfigs.attach_object) {
            Json::Value gpsInfoStyle(Json::ValueType::objectValue);
            gpsInfoStyle["text_size"] = gpsTextConfig.text_size;
            gpsInfoStyle["fill_color"] = "#FFFFFFFF";
            metaBuilder.streamStyle(gpsInfoStyle);

            int objectConfigsCnt = _commConfig.objectConfigs.size();
            for (int i = 0; i < objectConfigsCnt; ++i) {
                RsXvizObjectConfig objectConfig = _commConfig.objectConfigs[i];
                Json::Value classStyle(Json::ValueType::objectValue);
                std::string color = objectConfig.color.fromVecToColorString();
                classStyle["fill_color"] = color;
                classStyle["stroke_color"] = color.substr(0, color.size() - 2);
                metaBuilder.classStyle(objectConfig.type, classStyle);
            }
        }
        else {
            Json::Value gpsInfoStyle(Json::ValueType::objectValue);
            std::string color = gpsTextConfig.color.fromVecToColorString();
            gpsInfoStyle["text_size"] = gpsTextConfig.text_size;
            gpsInfoStyle["fill_color"] = color;
            gpsInfoStyle["stroke_color"] = color.substr(0, color.size() - 2);
            metaBuilder.streamStyle(gpsInfoStyle);
        }

        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("text");
    }

    // Stream: grid_circle
    {
        metaBuilder.stream("/grids/grid_circles");
        metaBuilder.category("primitive");
        Json::Value gridCircleStyle(Json::ValueType::objectValue);
        std::string color =
        _commConfig.lidarGridConfig.grid_circle_color.fromVecToColorString();
        gridCircleStyle["color"] = color;
        gridCircleStyle["filled"] = false;
        gridCircleStyle["stroke_color"] = color.substr(0, color.size() - 2);
        metaBuilder.streamStyle(gridCircleStyle);

        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("circle");
    }

    // Stream: roi
    if (_commConfig.enablesConfig.roiEnables.rois_enable) {
        size_t fillRoisCnt = _commConfig.fillRoisConfig.roiInfos.size();

        for (size_t i = 0; i < fillRoisCnt; ++i) {
            std::string stream_id = "/roi/" + std::to_string(i) + "/polygons";

            metaBuilder.stream(stream_id);
            metaBuilder.category("primitive");
            metaBuilder.coordinate("VEHICLE_RELATIVE");
            Json::Value freespaceStyle(Json::ValueType::objectValue);
            freespaceStyle["extruded"] = true;
            std::string color =
            _commConfig.fillRoisConfig.roiColors[i].fromVecToColorString();
            freespaceStyle["fill_color"] = color;
            metaBuilder.streamStyle(freespaceStyle);
            metaBuilder.transformMatrix(transform);
            metaBuilder.type("polygon");
        }
    }

    // Stream: freespace
    if (_commConfig.enablesConfig.freespacesEnables.freespaces_enable) {
        metaBuilder.stream("/environment/freespace");
        metaBuilder.category("primitive");
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        Json::Value freespaceStyle(Json::ValueType::objectValue);
        freespaceStyle["extruded"] = true;
        std::string color = _commConfig.freespaceConfigs.fromVecToColorString();
        freespaceStyle["fill_color"] = color;
        metaBuilder.streamStyle(freespaceStyle);
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polygon");
    }

    // Stream: robosense lanes
    if (_commConfig.enablesConfig.lanesEnables.lanes_enable) {
        metaBuilder.stream("/environment/lanes");
        metaBuilder.category("primitive");
        Json::Value laneStyle(Json::ValueType::objectValue);
        std::string color = _commConfig.laneConfigs.fromVecToColorString();
        laneStyle["fill_color"] = color;
        laneStyle["stroke_color"] = color.substr(0, color.size() - 2);
        metaBuilder.streamStyle(laneStyle);
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polyline");
    }

    // Stream: robosense curbs
    if (_commConfig.enablesConfig.curbsEnables.curbs_enable) {
        metaBuilder.stream("/environment/curbs");
        metaBuilder.category("primitive");
        Json::Value curbStyle(Json::ValueType::objectValue);
        std::string color = _commConfig.curbConfigs.fromVecToColorString();
        curbStyle["fill_color"] = color;
        curbStyle["stroke_color"] = color.substr(0, color.size() - 2);
        metaBuilder.streamStyle(curbStyle);
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("polyline");
    }

    // Stream:
    if (_commConfig.enablesConfig.pointCloudEnables.pointcloud_enable) {
        metaBuilder.stream("/lidar/points");
        metaBuilder.category("primitive");
        Json::Value lidarPointsStyle(Json::ValueType::objectValue);
        lidarPointsStyle["fill_color"] = "#00a";
        lidarPointsStyle["radius_pixels"] = _commConfig.colorMapConfig.pointSize;
        metaBuilder.streamStyle(lidarPointsStyle);
        metaBuilder.coordinate("VEHICLE_RELATIVE");
        metaBuilder.transformMatrix(transform);
        metaBuilder.type("point");
    }

    // Streams:
    if (_commConfig.enablesConfig.mapEnables.maps_enable) {
        size_t fillMapsCnt = _commConfig.fillMapsConfig.maps.size();

        for (size_t i = 0; i < fillMapsCnt; ++i) {
            std::string map_streamid = "/map/" + std::to_string(i) + "/points";

            metaBuilder.stream(map_streamid);
            metaBuilder.category("primitive");
            Json::Value lidarPointsStyle(Json::ValueType::objectValue);
            lidarPointsStyle["fill_color"] = "#00a";
            lidarPointsStyle["radius_pixels"] =
            _commConfig.mapColorMapConfig.pointSize;
            metaBuilder.streamStyle(lidarPointsStyle);
            metaBuilder.coordinate("VEHICLE_RELATIVE");
            metaBuilder.transformMatrix(transform);
            metaBuilder.type("point");
        }
    }

    // log_info:
    Json::Value log_info(Json::ValueType::objectValue);
    log_info["start_time"] = _logStartTime;
    log_info["end_time"] = _logEndTime;
    metaBuilder.loginfo(log_info);

    // data
    xviz["data"] = metaBuilder.getMetadata();

    // json xviz
    kittiMetadata["xviz"] = xviz;

    // json data
    Json::FastWriter jsonWriter;
    std::string jsonData = jsonWriter.write(kittiMetadata);
    int jsonDataSize = jsonData.size();
    _metadata->gltfHeader[xvizGLTF::GLTF_OFFSET::GLTF_JSON_SIZE] = jsonDataSize;
    _metadata->jsonData.resize(jsonDataSize);
    memcpy(_metadata->jsonData.data(), jsonData.data(), jsonData.size());

    // std::cout << "jsonData = " << jsonData << std::endl;

    // json padding
    int jsonPaddingSize = jsonDataSize % 4 != 0 ? 4 - jsonDataSize % 4 : 0;
    _metadata->jsonPadding.clear();
    if (jsonPaddingSize > 0) {
        _metadata->jsonPadding.resize(jsonPaddingSize, '\0');
    }

    // binary data
    _metadata->binaryData.clear();
    _metadata->binaryHeader[xvizGLTF::GLTF_OFFSET::GLTF_BINARY_SIZE] = 0;

    // total gltf size: binary data size = 0
    _metadata->gltfHeader[xvizGLTF::GLTF_OFFSET::GLTF_FILE_SIZE] =
    (xvizGLTF::GLTF_OFFSET::GLTF_FIXED_HEADER_SIZE + jsonDataSize +
    jsonPaddingSize);

    return _metadata;
}

// Make Robosense Self Define Data
xvizGLTF::Ptr RsWebsocketSerialize::makeRobosenseMessageData(const robosense::perception::RsPerceptionMsg::Ptr &msg) {
    // std::cout << "makeRobosenseMessageData A" << std::endl;

    // Computer All Object's Corners/Polygons And Some Common Info.
    const int totalObjectsCnt = msg->rs_lidar_result_ptr->objects.size();
    // Filter Nan PointCloud
    const int totalPointCloudCnt1 = msg->rs_lidar_result_ptr->scan_ptr->size();
    robosense::RsPointCloudGPT::Ptr filterPointCloudPtr(new robosense::RsPointCloudGPT);
    filterPointCloudPtr->points.reserve(totalPointCloudCnt1);
    int totalPointCloudCnt2 = 0;
    for (int i = 0; i < totalPointCloudCnt1; ++i) {
        const auto &point = msg->rs_lidar_result_ptr->scan_ptr->points[i];
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) || std::isnan(point.intensity)) {
            continue;
        }
        filterPointCloudPtr->points.emplace_back(point);
        totalPointCloudCnt2++;
    }
    msg->rs_lidar_result_ptr->scan_ptr = filterPointCloudPtr;
    const int totalPointCloudCnt = totalPointCloudCnt2;
    const std::vector<robosense::perception::Lane::Ptr> &lanes = msg->rs_lidar_result_ptr->lanes;
    const std::vector<robosense::perception::Roadedge::Ptr> &curbs = msg->rs_lidar_result_ptr->roadedges;
    const std::vector<RsVector3f> &freespaces = msg->rs_lidar_result_ptr->freespace_ptr->fs_pts;
    const std::vector<robosense::perception::Object::Ptr> &objects = msg->rs_lidar_result_ptr->objects;

    // double time0 =
    // std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();

    // make attention stream without json
    if (_commConfig.enablesConfig.attentionEnables.attention_enable_polygon ||
    _commConfig.enablesConfig.attentionEnables.attention_enable_polyhedral) {
        int totalAttentionObjectsCnt = msg->rs_lidar_result_ptr->attention_objects.size();
        std::vector<robosense::perception::Object::Ptr> attention_objects = msg->rs_lidar_result_ptr->attention_objects;

        for (int i = 0; i < totalAttentionObjectsCnt; ++i) {
            const robosense::perception::Object::Ptr &roboObj = attention_objects[i];
            if (roboObj->core_infos_.attention_type != robosense::perception::AttentionType::ATTENTION) {
                attention_objects.erase(attention_objects.begin() + i);
                --totalAttentionObjectsCnt;
                --i;
                continue;
            }

            if (roboObj->supplement_infos_.mode != robosense::perception::ModeType::BSD) {
                attention_objects.erase(attention_objects.begin() + i);
                --totalAttentionObjectsCnt;
                --i;
                continue;
            }

            if (roboObj->supplement_infos_.polygon.size() < 2) {
                attention_objects.erase(attention_objects.begin() + i);
                --totalAttentionObjectsCnt;
                --i;
                continue;
            }
        }

        std::vector<std::vector<RsXvizPoint3D>> attentionObjectsPolygons2;
        makeSerializeAttentionObjectPolygons(attention_objects, attentionObjectsPolygons2);

        if (_commConfig.enablesConfig.attentionEnables.attention_enable_polyhedral &&
        _commConfig.enablesConfig.attentionEnables.attention_enable_polygon) {
            _attentionpolyhedral_offset = 0;

            const static std::string attention_polyhedral_streamid = "/attentions/attention_polyhedral";
            const static std::string attention_polyhedral_type = "polygons";
            const static std::string polyhedral_end = "]}";

            std::string polyhedral_start = "\"" + attention_polyhedral_streamid +
            "\":{\"" + attention_polyhedral_type +
            "\":[";
            memcpy(_attentionPolyhedralBuffer.data() + _attentionpolyhedral_offset,
                   polyhedral_start.data(), polyhedral_start.size());
            _attentionpolyhedral_offset += polyhedral_start.size();

            _attentionpolygon_offset = 0;

            const static std::string attention_polygon_streamid = "/attentions/attention_polygon";
            const static std::string attention_polygon_type = "polylines";
            const static std::string polygon_end = "]}";

            std::string polygon_start = "\"" + attention_polygon_streamid + "\":{\"" +
            attention_polygon_type + "\":[";
            memcpy(_attentionPolygonBuffer.data() + _attentionpolygon_offset,
                   polygon_start.data(), polygon_start.size());
            _attentionpolygon_offset += polygon_start.size();

            std::string json_array_polyhedral;
            std::string json_array_polygon;
            int totalAttentionObjectsCnt1 = totalAttentionObjectsCnt - 1;
            for (int iterObj = 0; iterObj < totalAttentionObjectsCnt; ++iterObj) {
                const robosense::perception::Object::Ptr &roboObj = attention_objects[iterObj];

                json_array_polyhedral.clear();
                json_array_polygon.clear();

                std::string polyhedral_vertices;
                std::string polygon_vertices;
                makeSerializeAttentionCubeBox(attentionObjectsPolygons2[iterObj],
                polyhedral_vertices);
                makeSerializeAttentionGridBox(attentionObjectsPolygons2[iterObj],
                polygon_vertices);

                json_array_polyhedral += "{\"base\":{\"object_id\":\"" +
                std::to_string(roboObj->core_infos_.tracker_id) +
                "\",\"style\":{\"height\":" +
                std::to_string(roboObj->core_infos_.size.z) + "}}," +
                polyhedral_vertices + "}";
                json_array_polygon += "{\"base\":{\"object_id\":\"" +
                std::to_string(roboObj->core_infos_.tracker_id) +
                "\"}," + polygon_vertices + "}";

                if (iterObj != totalAttentionObjectsCnt1) {
                    json_array_polyhedral += ",";
                    json_array_polygon += ",";
                }

                memcpy(_attentionPolyhedralBuffer.data() + _attentionpolyhedral_offset,
                       json_array_polyhedral.data(), json_array_polyhedral.size());
                _attentionpolyhedral_offset += json_array_polyhedral.size();

                memcpy(_attentionPolygonBuffer.data() + _attentionpolygon_offset,
                       json_array_polygon.data(), json_array_polygon.size());
                _attentionpolygon_offset += json_array_polygon.size();
            }

            memcpy(_attentionPolyhedralBuffer.data() + _attentionpolyhedral_offset,
                   polyhedral_end.data(), polyhedral_end.size());
            _attentionpolyhedral_offset += polyhedral_end.size();

            memcpy(_attentionPolygonBuffer.data() + _attentionpolygon_offset,
                   polygon_end.data(), polygon_end.size());
            _attentionpolygon_offset += polygon_end.size();
        }
        else if (_commConfig.enablesConfig.attentionEnables.attention_enable_polygon) {
            _attentionpolygon_offset = 0;
            _attentionPolygonBuffer.resize(_defaultBufferCnt);
            const static std::string attention_polygon_streamid = "/attentions/attention_polygon";
            const static std::string attention_polygon_type = "polylines";
            const static std::string polygon_end = "]}";

            std::string polygon_start = "\"" + attention_polygon_streamid + "\":{\"" +
            attention_polygon_type + "\":[";
            memcpy(_attentionPolygonBuffer.data() + _attentionpolygon_offset,
                   polygon_start.data(), polygon_start.size());
            _attentionpolygon_offset += polygon_start.size();

            std::string json_array_polygon;
            int totalAttentionObjectsCnt1 = totalAttentionObjectsCnt - 1;
            for (int iterObj = 0; iterObj < totalAttentionObjectsCnt; ++iterObj) {
                const robosense::perception::Object::Ptr &roboObj = attention_objects[iterObj];
                json_array_polygon.clear();

                std::string polygon_vertices;
                makeSerializeAttentionGridBox(attentionObjectsPolygons2[iterObj],polygon_vertices);

                json_array_polygon += "{\"base\":{\"object_id\":\"" +
                std::to_string(roboObj->core_infos_.tracker_id) + "\"}," + polygon_vertices + "}";
                if (iterObj != totalAttentionObjectsCnt1) {
                    json_array_polygon += ",";
                }

                memcpy(_attentionPolygonBuffer.data() + _attentionpolygon_offset,
                       json_array_polygon.data(), json_array_polygon.size());
                _attentionpolygon_offset += json_array_polygon.size();
            }

            memcpy(_attentionPolygonBuffer.data() + _attentionpolygon_offset,
                   polygon_end.data(), polygon_end.size());
            _attentionpolygon_offset += polygon_end.size();
        }
        else if (_commConfig.enablesConfig.attentionEnables.attention_enable_polyhedral) {
            _attentionpolyhedral_offset = 0;
            _attentionPolyhedralBuffer.resize(_defaultBufferCnt);
            const static std::string attention_polyhedral_streamid = "/attentions/attention_polyhedral";
            const static std::string attention_polyhedral_type = "polygons";
            const static std::string polyhedral_end = "]}";

            std::string polyhedral_start = "\"" + attention_polyhedral_streamid +
            "\":{\"" + attention_polyhedral_type + "\":[";
            memcpy(_attentionPolyhedralBuffer.data() + _attentionpolyhedral_offset,
                   polyhedral_start.data(), polyhedral_start.size());
            _attentionpolyhedral_offset += polyhedral_start.size();

            std::string json_array_polyhedral;
            int totalAttentionObjectsCnt1 = totalAttentionObjectsCnt - 1;
            for (int iterObj = 0; iterObj < totalAttentionObjectsCnt; ++iterObj) {
                const robosense::perception::Object::Ptr &roboObj = attention_objects[iterObj];

                json_array_polyhedral.clear();

                std::string polyhedral_vertices;
                std::string polygon_vertices;
                makeSerializeAttentionCubeBox(attentionObjectsPolygons2[iterObj],polyhedral_vertices);
                makeSerializeAttentionGridBox(attentionObjectsPolygons2[iterObj],polygon_vertices);

                json_array_polyhedral += "{\"base\":{\"object_id\":\"" +
                std::to_string(roboObj->core_infos_.tracker_id) +
                "\",\"style\":{\"height\":" +
                std::to_string(roboObj->core_infos_.size.z) + "}}," +
                polyhedral_vertices + "}";

                if (iterObj != totalAttentionObjectsCnt1) {
                    json_array_polyhedral += ",";
                }

                memcpy(_attentionPolyhedralBuffer.data() + _attentionpolyhedral_offset,
                       json_array_polyhedral.data(), json_array_polyhedral.size());
                _attentionpolyhedral_offset += json_array_polyhedral.size();
            }

            memcpy(_attentionPolyhedralBuffer.data() + _attentionpolyhedral_offset,
                   polyhedral_end.data(), polyhedral_end.size());
            _attentionpolyhedral_offset += polyhedral_end.size();
        }
    }

    // std::cout << "makeRobosenseMessageData B" << std::endl;
    // double time1 =
    // std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();

    // Make Object Stream(s) without JSON
    {
        std::vector<std::vector<RsXvizPoint3D>> objectsCorners2;
        std::vector<std::vector<RsXvizPoint3D>> objectsPolygons2;
        makeSerializeObjectCorners(objects, objectsCorners2);
        makeSerializeObjectPolygons(objects, objectsPolygons2);

        _objectbox_offset = 0;
        const static std::string objectbox_streamid = "/objects/object_boxes";
        const static std::string objectbox_type = "polylines";

        _objectcube_offset = 0;
        const static std::string objectcube_streamid = "/objects/object_cubes";
        const static std::string objectcube_type = "polygons";

        _objectpolygon_offset = 0;
        const static std::string objectpolygon_streamid = "/objects/object_polygon";
        const static std::string objectpolygon_type = "polylines";

        _objectpolyhedral_offset = 0;
        const static std::string objectpolyhedral_streamid = "/objects/object_polyhedral";
        const static std::string objectpolyhedral_type = "polygons";

        _trackingpoint_offset = 0;
        const static std::string trackingpoint_streamid = "/objects/tracking_point";
        const static std::string trackingpoint_type = "circles";

        _trajectory_offset = 0;
        const static std::string trajectory_streamid = "/objects/trajectory";
        const static std::string trajectory_type = "circles";

        _velocitydir_offset = 0;
        const static std::string velocitydir_streamid = "/objects/velocity_dir";
        const static std::string velocitydir_type = "polylines";

        _boxinfo_offset = 0;
        const static std::string boxinfo_streamid = "/texts/box_info";
        const static std::string boxinfo_type = "texts";

        _labelinfo_offset = 0;
        const static std::string labelinfo_streamid = "/texts/label_info";
        const static std::string labelinfo_type = "texts";

        _trackinfo_offset = 0;
        const static std::string trackinfo_streamid = "/texts/track_info";
        const static std::string trackinfo_type = "texts";

        _gpsinfo_offset = 0;
        const static std::string gpsinfo_streamid = "/texts/gps_info";
        const static std::string gpsinfo_type = "texts";

        const static std::string common_end = "]}";
        const static std::string objectbox_start = "\"" + objectbox_streamid + "\":{\"" + objectbox_type + "\":[";
        const static std::string objectcube_start = "\"" + objectcube_streamid + "\":{\"" + objectcube_type + "\":[";

        const static std::string objectpolygon_start =
        "\"" + objectpolygon_streamid + "\":{\"" + objectpolygon_type + "\":[";

        const static std::string objectpolyhedral_start =
        "\"" + objectpolyhedral_streamid + "\":{\"" + objectpolyhedral_type + "\":[";

        const static std::string trackingpoint_start =
        "\"" + trackingpoint_streamid + "\":{\"" + trackingpoint_type + "\":[";

        const static std::string trajectory_start = "\"" + trajectory_streamid + "\":{\"" + trajectory_type + "\":[";

        const static std::string velocitydir_start = "\"" + velocitydir_streamid + "\":{\"" + velocitydir_type + "\":[";

        const static std::string boxinfo_start = "\"" + boxinfo_streamid + "\":{\"" + boxinfo_type + "\":[";

        const static std::string labelinfo_start = "\"" + labelinfo_streamid + "\":{\"" + labelinfo_type + "\":[";

        const static std::string trackinfo_start = "\"" + trackinfo_streamid + "\":{\"" + trackinfo_type + "\":[";

        const static std::string gpsinfo_start = "\"" + gpsinfo_streamid + "\":{\"" + gpsinfo_type + "\":[";

        if (_commConfig.enablesConfig.objectEnables.object_enable_box) {
            memcpy(_objectBoxesBuffer.data() + _objectbox_offset, objectbox_start.data(), objectbox_start.size());
            _objectbox_offset += objectbox_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_cube) {
            memcpy(_objectCubesBuffer.data() + _objectcube_offset,
                   objectcube_start.data(), objectcube_start.size());
            _objectcube_offset += objectcube_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_polygon) {
            memcpy(_objectPolygonBuffer.data() + _objectpolygon_offset,
                   objectpolygon_start.data(), objectpolygon_start.size());
            _objectpolygon_offset += objectpolygon_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_polyhedral) {
            memcpy(_objectPolyhedralBuffer.data() + _objectpolyhedral_offset,
                   objectpolyhedral_start.data(), objectpolyhedral_start.size());
            _objectpolyhedral_offset += objectpolyhedral_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_trackingpoint) {
            memcpy(_trackingPointBuffer.data() + _trackingpoint_offset,
                   trackingpoint_start.data(), trackingpoint_start.size());
            _trackingpoint_offset += trackingpoint_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_trajectory) {
            memcpy(_trajectoryBuffer.data() + _trajectory_offset, trajectory_start.data(), trajectory_start.size());
            _trajectory_offset += trajectory_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_velocitydir) {
            memcpy(_velocityDirBuffer.data() + _velocitydir_offset,
                   velocitydir_start.data(), velocitydir_start.size());
            _velocitydir_offset += velocitydir_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_boxinfo) {
            memcpy(_boxInfoBuffer.data() + _boxinfo_offset, boxinfo_start.data(), boxinfo_start.size());
            _boxinfo_offset += boxinfo_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_labelinfo) {
            memcpy(_labelInfoBuffer.data() + _labelinfo_offset, labelinfo_start.data(), labelinfo_start.size());
            _labelinfo_offset += labelinfo_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_trackinfo) {
            memcpy(_trackInfoBuffer.data() + _trackinfo_offset, trackinfo_start.data(), trackinfo_start.size());
            _trackinfo_offset += trackinfo_start.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_gpsinfo) {
            memcpy(_gpsInfoBuffer.data() + _gpsinfo_offset, gpsinfo_start.data(), gpsinfo_start.size());
            _gpsinfo_offset += gpsinfo_start.size();
        }

        std::string json_array;
        int totalObjectsCnt1 = totalObjectsCnt - 1;
        for (int iterObj = 0; iterObj < totalObjectsCnt; ++iterObj) {
            const robosense::perception::Object::Ptr &roboObj = objects[iterObj];
            std::string type = fromTypeIDToType(static_cast<int>(roboObj->core_infos_.type));
            std::string id = std::to_string(roboObj->core_infos_.tracker_id);
            std::string height = std::to_string(roboObj->core_infos_.size.z);

            std::string base0 = std::string("{\"base\":{\"object_id\":\"") + id + "\"},";
            std::string base1 = std::string("{\"base\":{\"classes\":[\"") + type +
            "\"],\"object_id\":\"" + id + "\"},";
            std::string base2 = std::string("{\"base\":{\"classes\":[\"") + type +
            "\"],\"object_id\":\"" + id +
            "\",\"style\":{\"height\":" + height + "}},";

            std::string comm = (iterObj != totalObjectsCnt1 ? "}," : "}");

            // object_boxs
            if (_commConfig.enablesConfig.objectEnables.object_enable_box) {
                json_array.clear();
                makeSerializeGridBox(objectsCorners2[iterObj], json_array);
                json_array += comm;
                memcpy(_objectBoxesBuffer.data() + _objectbox_offset, base1.data(), base1.size());
                _objectbox_offset += base1.size();
                memcpy(_objectBoxesBuffer.data() + _objectbox_offset, json_array.data(), json_array.size());
                _objectbox_offset += json_array.size();
            }

            // object_cubes
            if (_commConfig.enablesConfig.objectEnables.object_enable_cube) {
                json_array.clear();
                makeSerializeCubeBox(objectsCorners2[iterObj], json_array);
                json_array += comm;
                memcpy(_objectCubesBuffer.data() + _objectcube_offset, base2.data(), base2.size());
                _objectcube_offset += base2.size();
                memcpy(_objectCubesBuffer.data() + _objectcube_offset, json_array.data(), json_array.size());
                _objectcube_offset += json_array.size();
            }

            // object_polygons
            if (_commConfig.enablesConfig.objectEnables.object_enable_polygon) {
                json_array.clear();
                makeSerializeGridBox(objectsPolygons2[iterObj], json_array);
                json_array += comm;
                memcpy(_objectPolygonBuffer.data() + _objectpolygon_offset, base1.data(), base1.size());
                _objectpolygon_offset += base1.size();
                memcpy(_objectPolygonBuffer.data() + _objectpolygon_offset, json_array.data(), json_array.size());
                _objectpolygon_offset += json_array.size();
            }

            // object_polyhedral
            if (_commConfig.enablesConfig.objectEnables.object_enable_polyhedral) {
                json_array.clear();
                makeSerializeCubeBox(objectsPolygons2[iterObj], json_array);
                json_array += comm;
                memcpy(_objectPolyhedralBuffer.data() + _objectpolyhedral_offset, base2.data(), base2.size());
                _objectpolyhedral_offset += base2.size();
                memcpy(_objectPolyhedralBuffer.data() + _objectpolyhedral_offset,
                       json_array.data(), json_array.size());
                _objectpolyhedral_offset += json_array.size();
            }

            // tracking_point
            if (_commConfig.enablesConfig.objectEnables.object_enable_trackingpoint) {
                json_array.clear();
                if (_commConfig.trackPointConfigs.attach_object) {
                    json_array = base1 + "\"center\":[" +
                    std::to_string(roboObj->core_infos_.center.x) + "," +
                    std::to_string(roboObj->core_infos_.center.y) + "," +
                    std::to_string(roboObj->core_infos_.center.z) +
                    "],\"radius\":" + std::to_string(0.15) + comm;
                }
                else {
                    json_array = base0 + "\"center\":[" +
                    std::to_string(roboObj->core_infos_.center.x) + "," +
                    std::to_string(roboObj->core_infos_.center.y) + "," +
                    std::to_string(roboObj->core_infos_.center.z) +
                    "],\"radius\":" + std::to_string(0.15) + comm;
                }
                memcpy(_trackingPointBuffer.data() + _trackingpoint_offset, json_array.data(), json_array.size());
                _trackingpoint_offset += json_array.size();
            }

            // trajectory
            if (_commConfig.enablesConfig.objectEnables.object_enable_trajectory) {
                json_array.clear();
                std::vector<std::string> json_array_trajs;
                makeSerializeTrajectory(roboObj->supplement_infos_.trajectory, json_array_trajs);

                int trajectoryCnt = json_array_trajs.size();
                int trajectoryCnt1 = trajectoryCnt - 1;
                for (int iterHistory = 0; iterHistory < trajectoryCnt; ++iterHistory) {
                    std::string comm2 = ((iterObj == totalObjectsCnt1 && iterHistory == trajectoryCnt1) ? "}" : "},");

                    if (_commConfig.trajectoryConfigs.attach_object) {
                        json_array = std::string("{\"base\":{\"classes\":[\"") + type +
                        "\"],\"object_id\":\"" + id + "_" +
                        std::to_string(iterHistory) + "\"}," +
                        json_array_trajs[iterHistory] +
                        ",\"radius\":" + std::to_string(0.15) + comm2;
                    }
                    else {
                        json_array = std::string("{\"base\":{\"object_id\":\"") + id + "_" +
                        std::to_string(iterHistory) + "\"}," +
                        json_array_trajs[iterHistory] +
                        ",\"radius\":" + std::to_string(0.15) + comm2;
                    }
                    memcpy(_trajectoryBuffer.data() + _trajectory_offset, json_array.data(), json_array.size());
                    _trajectory_offset += json_array.size();
                }
            }

            // velocity_dir
            if (_commConfig.enablesConfig.objectEnables.object_enable_velocitydir) {
                float velocity = roboObj->core_infos_.velocity.norm();

                RsXvizPoint3D start, end;
                if (velocity > 1e-1) {
                    // Set Z-Axis Direction Velocity is 0
                    // Eigen::Vector3f velocityOne = roboObj->core_infos_.velocity;
                    RsVector3f &velocityOne = roboObj->core_infos_.velocity;
                    velocityOne.z = 0;
                    velocityOne.normalize();

                    velocityOne *= std::sqrt(velocity);

                    start.x = roboObj->core_infos_.center.x;
                    start.y = roboObj->core_infos_.center.y;
                    start.z = roboObj->core_infos_.center.z;

                    end.x = roboObj->core_infos_.center.x + velocityOne.x;
                    end.y = roboObj->core_infos_.center.y + velocityOne.y;
                    end.z = roboObj->core_infos_.center.z + velocityOne.z;
                }
                else {
                    start.x = roboObj->core_infos_.center.x;
                    start.y = roboObj->core_infos_.center.y;
                    start.z = roboObj->core_infos_.center.z;

                    end.x = roboObj->core_infos_.center.x + velocity;
                    end.y = roboObj->core_infos_.center.y + velocity;
                    end.z = roboObj->core_infos_.center.z + velocity;
                }
                start.updateJsonArray();
                end.updateJsonArray();

                if (_commConfig.velocityDirConfig.attach_object) {
                    json_array = base1 + "\"vertices\":[" + start.json_array + "," +
                    end.json_array + "]" + comm;
                }
                else {
                    json_array = base0 + "\"vertices\":[" + start.json_array + "," +
                    end.json_array + "]" + comm;
                }
                memcpy(_velocityDirBuffer.data() + _velocitydir_offset, json_array.data(), json_array.size());
                _velocitydir_offset += json_array.size();
            }

            // box_info / label_info / track_info / gps_info
            {
                std::string textBase0, textBase1;
                if (_commConfig.boxTextConfigs.attach_object || _commConfig.labelTextConfigs.attach_object ||
                _commConfig.trackTextConfigs.attach_object || _commConfig.gpsTextConfigs.attach_object) {
                    textBase1 = std::string("{\"base\":{\"classes\":[\"") + type +
                    "\"],\"object_id\":\"" + id + "\"},\"position\":[" +
                    std::to_string(roboObj->core_infos_.center.x) + "," +
                    std::to_string(roboObj->core_infos_.center.y) + "," +
                    std::to_string(roboObj->core_infos_.center.z +
                    roboObj->core_infos_.size.z / 2.0) +
                    "],\"text\":\"";
                }

                if (!_commConfig.labelTextConfigs.attach_object || !_commConfig.labelTextConfigs.attach_object ||
                    !_commConfig.trackTextConfigs.attach_object || !_commConfig.gpsTextConfigs.attach_object) {
                    textBase0 = std::string("{\"base\":{\"object_id\":\"") + id +
                    "\"},\"position\":[" +
                    std::to_string(roboObj->core_infos_.center.x) + "," +
                    std::to_string(roboObj->core_infos_.center.y) + "," +
                    std::to_string(roboObj->core_infos_.center.z +
                    roboObj->core_infos_.size.z / 2.0) +
                    "],\"text\":\"";
                }

                if (_commConfig.enablesConfig.objectEnables.object_enable_boxinfo) {
                    std::string box_info =
                    num2str<float>(roboObj->core_infos_.anchor.norm(), 2) +
                    std::string("(") +
                    num2str<float>(roboObj->core_infos_.size.x, 1) +
                    std::string(",") +
                    num2str<float>(roboObj->core_infos_.size.y, 1) +
                    std::string(",") +
                    num2str<float>(roboObj->core_infos_.size.z, 1) + std::string(")");

                    if (_commConfig.boxTextConfigs.attach_object) {
                        json_array = textBase1 + box_info + "\"" + comm;
                    }
                    else {
                        json_array = textBase0 + box_info + "\"" + comm;
                    }
                    memcpy(_boxInfoBuffer.data() + _boxinfo_offset, json_array.data(), json_array.size());
                    _boxinfo_offset += json_array.size();
                }

                if (_commConfig.enablesConfig.objectEnables.object_enable_labelinfo) {
                    std::string label_info = type + std::string(">>") +
                    num2str<float>(roboObj->core_infos_.type_confidence, 2);

                    if (_commConfig.labelTextConfigs.attach_object) {
                        json_array = textBase1 + label_info + "\"" + comm;
                    }
                    else {
                        json_array = textBase0 + label_info + "\"" + comm;
                    }

                    memcpy(_labelInfoBuffer.data() + _labelinfo_offset, json_array.data(), json_array.size());
                    _labelinfo_offset += json_array.size();
                }

                if (_commConfig.enablesConfig.objectEnables.object_enable_trackinfo) {
                    std::string track_info =
                    std::string("<") +
                    num2str<float>(roboObj->core_infos_.tracker_id, 0) +
                    std::string("> ") +
                    num2str<float>(roboObj->core_infos_.velocity.norm() * 3.6, 2) +
                    std::string(" km/h");

                    if (_commConfig.trackTextConfigs.attach_object) {
                        json_array = textBase1 + track_info + "\"" + comm;
                    }
                    else {
                        json_array = textBase0 + track_info + "\"" + comm;
                    }
                    memcpy(_trackInfoBuffer.data() + _trackinfo_offset, json_array.data(), json_array.size());
                    _trackinfo_offset += json_array.size();
                }

                if (_commConfig.enablesConfig.objectEnables.object_enable_gpsinfo) {
                    std::string gps_info =
                    std::string("<") +
                    num2str<double>(roboObj->supplement_infos_.gps_longtitude, 7) +
                    ", " +
                    num2str<double>(roboObj->supplement_infos_.gps_latitude, 7) +
                    ", " +
                    num2str<double>(roboObj->supplement_infos_.gps_altitude, 7) +
                    std::string(">");

                    if (_commConfig.gpsTextConfigs.attach_object) {
                        json_array = textBase1 + gps_info + "\"" + comm;
                    }
                    else {
                        json_array = textBase0 + gps_info + "\"" + comm;
                    }
                    memcpy(_gpsInfoBuffer.data() + _gpsinfo_offset, json_array.data(), json_array.size());
                    _gpsinfo_offset += json_array.size();
                }
            }
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_box) {
            memcpy(_objectBoxesBuffer.data() + _objectbox_offset, common_end.data(), common_end.size());
            _objectbox_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_cube) {
            memcpy(_objectCubesBuffer.data() + _objectcube_offset, common_end.data(), common_end.size());
            _objectcube_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_polygon) {
            memcpy(_objectPolygonBuffer.data() + _objectpolygon_offset, common_end.data(), common_end.size());
            _objectpolygon_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_polyhedral) {
            memcpy(_objectPolyhedralBuffer.data() + _objectpolyhedral_offset,
                   common_end.data(), common_end.size());
            _objectpolyhedral_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_trackingpoint) {
            memcpy(_trackingPointBuffer.data() + _trackingpoint_offset, common_end.data(), common_end.size());
            _trackingpoint_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_trajectory) {
            memcpy(_trajectoryBuffer.data() + _trajectory_offset, common_end.data(), common_end.size());
            _trajectory_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_velocitydir) {
            memcpy(_velocityDirBuffer.data() + _velocitydir_offset, common_end.data(), common_end.size());
            _velocitydir_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_boxinfo) {
            memcpy(_boxInfoBuffer.data() + _boxinfo_offset, common_end.data(), common_end.size());
            _boxinfo_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_labelinfo) {
            memcpy(_labelInfoBuffer.data() + _labelinfo_offset, common_end.data(), common_end.size());
            _labelinfo_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_trackinfo) {
            memcpy(_trackInfoBuffer.data() + _trackinfo_offset, common_end.data(), common_end.size());
            _trackinfo_offset += common_end.size();
        }

        if (_commConfig.enablesConfig.objectEnables.object_enable_gpsinfo) {
            memcpy(_gpsInfoBuffer.data() + _gpsinfo_offset, common_end.data(), common_end.size());
            _gpsinfo_offset += common_end.size();
        }
    }
    // std::cout << "makeRobosenseMessageData C" << std::endl;
    // Make Freespace
    if (_commConfig.enablesConfig.freespacesEnables.freespaces_enable) {
        const static std::string freespaces_streamid = "/environment/freespace";
        const static std::string freespace_type = "polygons";
        makeSerializeFreespace(freespaces, freespaces_streamid, freespace_type, _freespaceBuffer);
    }

    // lane streams without json
    if (_commConfig.enablesConfig.lanesEnables.lanes_enable) {
        const static std::string lanes_streamid = "/environment/lanes";
        const static std::string lanes_type = "polylines";
        makeSerializeLanes(lanes, lanes_streamid, lanes_type, _lanesBuffer);
    }

    // curb stream without json
    if (_commConfig.enablesConfig.curbsEnables.curbs_enable) {
        const static std::string curbs_streamid = "/environment/curbs";
        const static std::string curbs_type = "polylines";
        makeSerializeCurbs(curbs, curbs_streamid, curbs_type, _curbsBuffer);
    }
    // std::cout << "makeRobosenseMessageData D" << std::endl;
    // Use Package XVIZ Time as Tiemstamp
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>
    currentTime = std::chrono::time_point_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now());
    double timeStamp = currentTime.time_since_epoch().count();
    timeStamp /= 1000.0f;

    // Make Pose Stream Without Json
    {
        makeSerializePoseStream( "/vehicle_pose", timeStamp, _commConfig.mapOriginConfig.lon,
                                 _commConfig.mapOriginConfig.lat, _commConfig.mapOriginConfig.alt,
                                 msg->rs_lidar_result_ptr->global_pose_ptr->x,
                                 msg->rs_lidar_result_ptr->global_pose_ptr->y,
                                 msg->rs_lidar_result_ptr->global_pose_ptr->z,
                                 msg->rs_lidar_result_ptr->global_pose_ptr->roll,
                                 msg->rs_lidar_result_ptr->global_pose_ptr->pitch,
                                 msg->rs_lidar_result_ptr->global_pose_ptr->yaw, _poseStreamBuffers);
    }

    // Make GLTF Element: buffers Without Json

    {
        int firstBuffersByteLength = 0;
        if (_commConfig.enablesConfig.pointCloudEnables.pointcloud_enable) {
            firstBuffersByteLength += totalPointCloudCnt * (4 + 12);
        }

        if (_commConfig.enablesConfig.mapEnables.maps_enable) {
            size_t fillMapsCnt = _commConfig.fillMapsConfig.maps.size();
            for (size_t i = 0; i < fillMapsCnt; ++i) {
                firstBuffersByteLength += _mapPointCntBuffers[i] * (4 + 12);
            }
        }

        makeSerializeBuffers(firstBuffersByteLength, _buffersBuffers);
    }

    // std::cout << "makeRobosenseMessageData E" << std::endl;
    // Make GLTF Element: bufferViews without Json

    {
        std::vector<std::pair<int, int>> bufferViewsInfos;

        int bufferViewsOffset = 0;
        if (_commConfig.enablesConfig.pointCloudEnables.pointcloud_enable) {
            bufferViewsInfos.push_back(std::pair<int, int>(totalPointCloudCnt * 4, bufferViewsOffset));
            bufferViewsOffset += totalPointCloudCnt * 4;

            bufferViewsInfos.push_back(std::pair<int, int>(totalPointCloudCnt * 12, bufferViewsOffset));
            bufferViewsOffset += totalPointCloudCnt * 12;
        }

        if (_commConfig.enablesConfig.mapEnables.maps_enable) {
            size_t fillMapsCnt = _commConfig.fillMapsConfig.maps.size();
            for (size_t i = 0; i < fillMapsCnt; ++i) {
                bufferViewsInfos.push_back(std::pair<int, int>(_mapColorBuffers[i].size(), bufferViewsOffset));
                bufferViewsOffset += _mapColorBuffers[i].size();

                bufferViewsInfos.push_back(std::pair<int, int>(_mapPointBuffers[i].size(), bufferViewsOffset));
                bufferViewsOffset += _mapPointBuffers[i].size();
            }
        }

        makeSerializeBufferViews(bufferViewsInfos, _bufferViewsBuffers);
    }

    // std::cout << "makeRobosenseMessageData F" << std::endl;
    // Make GLTF Element: Accessors Without Json
    {
        std::vector<int> accessorsInfos;
        if (_commConfig.enablesConfig.pointCloudEnables.pointcloud_enable) {
            accessorsInfos.push_back(totalPointCloudCnt);
            accessorsInfos.push_back(totalPointCloudCnt);
        }

        if (_commConfig.enablesConfig.mapEnables.maps_enable) {
            size_t fillMapsCnt = _commConfig.fillMapsConfig.maps.size();

            for (size_t i = 0; i < fillMapsCnt; ++i) {
                accessorsInfos.push_back(_mapPointCntBuffers[i]);
                accessorsInfos.push_back(_mapPointCntBuffers[i]);
            }
        }

        if (accessorsInfos.size() > 0) {
            makeSerializeAccessors(accessorsInfos, _accessorsBuffer);
        }
    }

    // std::cout << "makeRobosenseMessageData G" << std::endl;
    // Make Frame XVIZ Data
    const static char object_start = '{';
    const static char object_end = '}';
    const static char json_spliter = ',';

    int jsonDataSize = 0;
    _message->jsonData.resize(2 * _defaultBufferCnt);  //
    char *bufferPtr = _message->jsonData.data();

    bufferPtr[jsonDataSize] = object_start;
    jsonDataSize += 1;

    // GLTF Element: accessors
    memcpy(bufferPtr + jsonDataSize, _accessorsBuffer.data(), _accessorsBuffer.size());
    jsonDataSize += _accessorsBuffer.size();

    bufferPtr[jsonDataSize] = json_spliter;
    jsonDataSize += 1;

    // GLTF Element: bufferViews
    memcpy(bufferPtr + jsonDataSize, _bufferViewsBuffers.data(), _bufferViewsBuffers.size());
    jsonDataSize += _bufferViewsBuffers.size();

    bufferPtr[jsonDataSize] = json_spliter;
    jsonDataSize += 1;

    // GLTF Element: buffers
    memcpy(bufferPtr + jsonDataSize, _buffersBuffers.data(), _buffersBuffers.size());
    jsonDataSize += _buffersBuffers.size();

    bufferPtr[jsonDataSize] = json_spliter;
    jsonDataSize += 1;

    // GLTF Element: meshes
    memcpy(bufferPtr + jsonDataSize, _meshesBuffers.data(), _meshesBuffers.size());
    jsonDataSize += _meshesBuffers.size();

    bufferPtr[jsonDataSize] = json_spliter;
    jsonDataSize += 1;

    // GLTF Element: xviz
    {
        std::string xviz_start =
        "\"xviz\":{\"type\":\"xviz/"
        "state_update\",\"data\":{\"update_type\":\"snapshot\",\"updates\":[{"
        "\"timestamp\":" +
        std::to_string(timeStamp) + ",";
        std::string xviz_end = "}]}}";

        memcpy(bufferPtr + jsonDataSize, xviz_start.data(), xviz_start.size());
        jsonDataSize += xviz_start.size();

        // Type: Pose
        memcpy(bufferPtr + jsonDataSize, _poseStreamBuffers.data(), _poseStreamBuffers.size());
        jsonDataSize += _poseStreamBuffers.size();

        bufferPtr[jsonDataSize] = json_spliter;
        jsonDataSize += 1;

        // Type: primitive
        std::string primitive_start = "\"primitives\":{";
        memcpy(bufferPtr + jsonDataSize, primitive_start.data(), primitive_start.size());
        jsonDataSize += primitive_start.size();

        // Primitive Stream: attention_polygon
        if (_commConfig.enablesConfig.attentionEnables.attention_enable_polygon) {
            memcpy(bufferPtr + jsonDataSize, _attentionPolygonBuffer.data(), _attentionpolygon_offset);
            jsonDataSize += _attentionpolygon_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: attention_polyhedral
        if (_commConfig.enablesConfig.attentionEnables.attention_enable_polyhedral) {
            memcpy(bufferPtr + jsonDataSize, _attentionPolyhedralBuffer.data(), _attentionpolyhedral_offset);
            jsonDataSize += _attentionpolyhedral_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: rois
        if (_commConfig.enablesConfig.roiEnables.rois_enable) {
            size_t fillRoisCnt = _commConfig.fillRoisConfig.roiInfos.size();
            for (size_t i = 0; i < fillRoisCnt; ++i) {
                memcpy(bufferPtr + jsonDataSize, _roisBuffer[i].data(), _roisBuffer[i].size());
                jsonDataSize += _roisBuffer[i].size();

                bufferPtr[jsonDataSize] = json_spliter;
                jsonDataSize += 1;
            }
        }

        // Primitive Stream: freespace
        if (_commConfig.enablesConfig.freespacesEnables.freespaces_enable) {
            memcpy(bufferPtr + jsonDataSize, _freespaceBuffer.data(), _freespaceBuffer.size());
            jsonDataSize += _freespaceBuffer.size();

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: Lanes
        if (_commConfig.enablesConfig.lanesEnables.lanes_enable) {
            memcpy(bufferPtr + jsonDataSize, _lanesBuffer.data(), _lanesBuffer.size());
            jsonDataSize += _lanesBuffer.size();

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: Curbs
        if (_commConfig.enablesConfig.curbsEnables.curbs_enable) {
            memcpy(bufferPtr + jsonDataSize, _curbsBuffer.data(), _curbsBuffer.size());
            jsonDataSize += _curbsBuffer.size();

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: grid_circle
        memcpy(bufferPtr + jsonDataSize, _gridCircleBuffer.data(), _gridCircleBuffer.size());
        jsonDataSize += _gridCircleBuffer.size();

        bufferPtr[jsonDataSize] = json_spliter;
        jsonDataSize += 1;

        // Primitive Stream: lidar_point
        if (_commConfig.enablesConfig.pointCloudEnables.pointcloud_enable) {
            memcpy(bufferPtr + jsonDataSize, _lidarPointStreamBuffers.data(), _lidarPointStreamBuffers.size());
            jsonDataSize += _lidarPointStreamBuffers.size();

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: map_point
        if (_commConfig.enablesConfig.mapEnables.maps_enable) {
            size_t fillMapsCnt = _commConfig.fillMapsConfig.maps.size();
            for (size_t i = 0; i < fillMapsCnt; ++i) {
                memcpy(bufferPtr + jsonDataSize, _mapPointStreamBuffers[i].data(),
                       _mapPointStreamBuffers[i].size());
                jsonDataSize += _mapPointStreamBuffers[i].size();

                bufferPtr[jsonDataSize] = json_spliter;
                jsonDataSize += 1;
            }
        }

        // Primitive Stream: object_boxs
        if (_commConfig.enablesConfig.objectEnables.object_enable_box) {
            memcpy(bufferPtr + jsonDataSize, _objectBoxesBuffer.data(), _objectbox_offset);
            jsonDataSize += _objectbox_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: object_cubes
        if (_commConfig.enablesConfig.objectEnables.object_enable_cube) {
            memcpy(bufferPtr + jsonDataSize, _objectCubesBuffer.data(), _objectcube_offset);
            jsonDataSize += _objectcube_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: object_polygon
        if (_commConfig.enablesConfig.objectEnables.object_enable_polygon) {
            memcpy(bufferPtr + jsonDataSize, _objectPolygonBuffer.data(), _objectpolygon_offset);
            jsonDataSize += _objectpolygon_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: object_polyhedral
        if (_commConfig.enablesConfig.objectEnables.object_enable_polyhedral) {
            memcpy(bufferPtr + jsonDataSize, _objectPolyhedralBuffer.data(), _objectpolyhedral_offset);
            jsonDataSize += _objectpolyhedral_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: tracking_point
        if (_commConfig.enablesConfig.objectEnables.object_enable_trackingpoint) {
            memcpy(bufferPtr + jsonDataSize, _trackingPointBuffer.data(), _trackingpoint_offset);
            jsonDataSize += _trackingpoint_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: trajectory
        if (_commConfig.enablesConfig.objectEnables.object_enable_trajectory) {
            memcpy(bufferPtr + jsonDataSize, _trajectoryBuffer.data(), _trajectory_offset);
            jsonDataSize += _trajectory_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: velocity_dir
        if (_commConfig.enablesConfig.objectEnables.object_enable_velocitydir) {
            memcpy(bufferPtr + jsonDataSize, _velocityDirBuffer.data(), _velocitydir_offset);
            jsonDataSize += _velocitydir_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: box_info
        if (_commConfig.enablesConfig.objectEnables.object_enable_boxinfo) {
            memcpy(bufferPtr + jsonDataSize, _boxInfoBuffer.data(), _boxinfo_offset);
            jsonDataSize += _boxinfo_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: label_info
        if (_commConfig.enablesConfig.objectEnables.object_enable_labelinfo) {
            memcpy(bufferPtr + jsonDataSize, _labelInfoBuffer.data(), _labelinfo_offset);
            jsonDataSize += _labelinfo_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: track_info
        if (_commConfig.enablesConfig.objectEnables.object_enable_trackinfo) {
            memcpy(bufferPtr + jsonDataSize, _trackInfoBuffer.data(), _trackinfo_offset);
            jsonDataSize += _trackinfo_offset;

            bufferPtr[jsonDataSize] = json_spliter;
            jsonDataSize += 1;
        }

        // Primitive Stream: gps_info
        if (_commConfig.enablesConfig.objectEnables.object_enable_gpsinfo) {
            memcpy(bufferPtr + jsonDataSize, _gpsInfoBuffer.data(), _gpsinfo_offset);
            jsonDataSize += _gpsinfo_offset;
        }

        bufferPtr[jsonDataSize] = object_end;
        jsonDataSize += 1;

        // xviz end
        memcpy(bufferPtr + jsonDataSize, xviz_end.data(), xviz_end.size());
        jsonDataSize += xviz_end.size();
    }

    bufferPtr[jsonDataSize] = object_end;
    jsonDataSize += 1;

    _message->jsonData.resize(jsonDataSize);
    _message->gltfHeader[xvizGLTF::GLTF_JSON_SIZE] = jsonDataSize;

    // std::cout << "Json Data = " << std::string(_message->jsonData.data(),
    // _message->jsonData.size()) << std::endl;

    // Json Padding
    int jsonPaddingSize = (jsonDataSize % 4 != 0 ? 4 - jsonDataSize % 4 : 0);
    _message->jsonPadding.clear();
    if (jsonPaddingSize > 0) {
        _message->jsonPadding.resize(jsonPaddingSize, '\0');
    }

    // double time2 =
    // std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();

    int totalBinarySize = 0;
    if (_commConfig.enablesConfig.pointCloudEnables.pointcloud_enable) {
        totalBinarySize += totalPointCloudCnt * (4 + 12);
    }

    if (_commConfig.enablesConfig.mapEnables.maps_enable) {
        size_t fillMapsCnt = _commConfig.fillMapsConfig.maps.size();
        for (size_t i = 0; i < fillMapsCnt; ++i) {
            totalBinarySize += _mapPointCntBuffers[i] * (4 + 12);
        }
    }

    // std::cout << "makeRobosenseMessageData H" << std::endl;
    _message->binaryHeader[xvizGLTF::GLTF_BINARY_SIZE] = totalBinarySize;

    // std::cout << "totalBinarySize = " << totalBinarySize << std::endl;

    _message->binaryData.resize(totalBinarySize, char(0));

    int binaryOffset = 0;
    if (_commConfig.enablesConfig.pointCloudEnables.pointcloud_enable) {
    // colors:
        if (!_commConfig.colorMapConfig.colorMapperEnable) {  // No Color Mapper
            for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                memcpy(_message->binaryData.data() + binaryOffset, _pointCloudColors, sizeof(_pointCloudColors));
                binaryOffset += 4;
            }
        }
        else {  // Color Mapper
            int factorCnt = 0;
            std::vector<float> lows;
            std::vector<float> highs;
            std::vector<float> alphas;

            int factorEnableCnt = _commConfig.colorMapConfig.factorsEnable.size();

            for (int i = 0; i < factorEnableCnt; ++i) {
                if (_commConfig.colorMapConfig.factorsEnable[i]) {
                    factorCnt += 1;
                    lows.push_back(_commConfig.colorMapConfig.factorsLow[i]);
                    highs.push_back(_commConfig.colorMapConfig.factorsUp[i]);
                    alphas.push_back(_commConfig.colorMapConfig.factorsAlphas[i]);
                }
            }

            if (factorCnt == 1 &&
            _commConfig.colorMapConfig.factorsEnable[static_cast<int>(
            RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_INTENSITY)] &&
            _commConfig.colorMapConfig.colorMapperType == RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_RVIZ_TYPE) {
                std::vector<st_ColorMapRGB_UCHAR> mappedColors(totalPointCloudCnt);
                for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                    mappedColors[iterPc].r = 255.0f;
                    float intensity = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc].intensity;
                    mappedColors[iterPc].g = std::min(50.0f, intensity) / 50.0f * 255.0f;
                    mappedColors[iterPc].b = 0.0f;
                    mappedColors[iterPc].a = 255.0f;
                }
                int totalColorBinarySize = totalPointCloudCnt * 4;
                memcpy(_message->binaryData.data() + binaryOffset, mappedColors.data(), totalColorBinarySize);
                binaryOffset += totalColorBinarySize;
            }
            else if (factorCnt == 1) {
                std::vector<float> srcArrays(totalPointCloudCnt, 0.0f);

                if (_commConfig.colorMapConfig.factorsEnable[static_cast<int>(
                RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_X)]) {
                    for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                    srcArrays[iterPc] = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc].x;
                    }
                }
                else if (_commConfig.colorMapConfig.factorsEnable[static_cast<int>(
                RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Y)]) {
                    for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                    srcArrays[iterPc] = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc].y;
                    }
                }
                else if (_commConfig.colorMapConfig.factorsEnable[static_cast<int>(
                RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Z)]) {
                    for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                    srcArrays[iterPc] = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc].z;
                    }
                }
                else if (_commConfig.colorMapConfig.factorsEnable[static_cast<int>(
                RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_INTENSITY)]) {
                    for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                    srcArrays[iterPc] = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc].intensity;
                    }
                }

                RSColorMapper colorMapper(_commConfig.colorMapConfig.colorMapperType, 100.0,
                (_commConfig.colorMapConfig.enableColorMapInvert ? false : true));

                // Default st_ColorMapRGB_UCHAR.a = 255
                std::vector<st_ColorMapRGB_UCHAR> mappedColors;
                st_ColorMapRGB_UCHAR bottomColor;

                colorMapper.m_applyColorMap(srcArrays, lows[0], highs[0], mappedColors,
                bottomColor);

                int totalColorBinarySize = totalPointCloudCnt * 4;
                memcpy(_message->binaryData.data() + binaryOffset, mappedColors.data(), totalColorBinarySize);
                binaryOffset += totalColorBinarySize;
            }
            else if (factorCnt > 1) {
                std::vector<std::vector<float>> srcArrays(factorCnt, std::vector<float>());

                float totalAlphas = 0;
                for (int i = 0; i < factorCnt; ++i) {
                    srcArrays[i].resize(totalPointCloudCnt, 0.0f);
                    totalAlphas += std::abs(alphas[i]);
                }

                for (int i = 0; i < factorCnt; ++i) {
                    alphas[i] = (totalAlphas < 0.01) ? (1.0f / factorCnt) : alphas[i] / totalAlphas;
                }

                for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                    int index = 0;

                    if (_commConfig.colorMapConfig.factorsEnable[static_cast<int>(
                    RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_X)]) {
                        srcArrays[index][iterPc] = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc].x;
                        index += 1;
                    }

                    if (_commConfig.colorMapConfig.factorsEnable[static_cast<int>(
                    RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Y)]) {
                        srcArrays[index][iterPc] = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc].y;
                        index += 1;
                    }

                    if (_commConfig.colorMapConfig.factorsEnable[static_cast<int>(
                    RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Z)]) {
                        srcArrays[index][iterPc] = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc].z;
                        index += 1;
                    }

                    if (_commConfig.colorMapConfig.factorsEnable[static_cast<int>(
                    RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_INTENSITY)]) {
                        srcArrays[index][iterPc] = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc].intensity;
                        index += 1;
                    }
                }

                RSColorMapper colorMapper(_commConfig.colorMapConfig.colorMapperType, 100.0,
                (_commConfig.colorMapConfig.enableColorMapInvert ? false : true));

                // Default st_ColorMapRGB_UCHAR.a = 255
                std::vector<st_ColorMapRGB_UCHAR> mappedColors;
                st_ColorMapRGB_UCHAR bottomColor;

                colorMapper.m_applyColorMap(srcArrays, alphas, lows, highs, mappedColors, bottomColor);

                int totalColorBinarySize = totalPointCloudCnt * 4;
                memcpy(_message->binaryData.data() + binaryOffset, mappedColors.data(), totalColorBinarySize);
                binaryOffset += totalColorBinarySize;
            }
            else {
                for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                    memcpy(_message->binaryData.data() + binaryOffset,
                           _pointCloudColors, sizeof(_pointCloudColors));
                    binaryOffset += 4;
                }
            }
        }

        // points
        {
            float x, y, z, intensity;
            const int float_x = sizeof(float);
            const int float_xy = sizeof(float) * 2;
            const int float_xyz = sizeof(float) * 3;
            for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                const auto &point = msg->rs_lidar_result_ptr->scan_ptr->points[iterPc];
                memcpy(_message->binaryData.data() + binaryOffset, &(point.x), sizeof(float));
                memcpy(_message->binaryData.data() + binaryOffset + float_x, &(point.y), sizeof(float));
                memcpy(_message->binaryData.data() + binaryOffset + float_xy, &(point.z), sizeof(float));
                binaryOffset += float_xyz;
            }
        }
    }
    // std::cout << "makeRobosenseMessageData I" << std::endl;
    // std::cout << "_mapPointBuffers size =  " << _mapPointBuffers.size() << ",
    // _mapColorBuffers size = " << _mapColorBuffers.size() << std::endl;

    // map
    if (_commConfig.enablesConfig.mapEnables.maps_enable) {
        size_t fillMapsCnt = _commConfig.fillMapsConfig.maps.size();
        for (size_t i = 0; i < fillMapsCnt; ++i) {
            // std::cout << "binaryData Size = " << _message->binaryData.size() << ",
            // _mapColor[" << i << "] Size = " << _mapColorBuffers[i].size() << ",
            // _mapPoint[" << i << "] Size = " << _mapPointBuffers[i].size() <<
            // std::endl;
            memcpy(_message->binaryData.data() + binaryOffset,
                   _mapColorBuffers[i].data(), _mapColorBuffers[i].size());
            binaryOffset += _mapColorBuffers[i].size();

            memcpy(_message->binaryData.data() + binaryOffset,
                   _mapPointBuffers[i].data(), _mapPointBuffers[i].size());
            binaryOffset += _mapPointBuffers[i].size();
        }
    }

    // std::cout << "makeRobosenseMessageData K" << std::endl;
    // Two Header + jsonData + jsonPadding + binaryData
    _message->gltfHeader[xvizGLTF::GLTF_FILE_SIZE] =
    (xvizGLTF::GLTF_FIXED_HEADER_SIZE + jsonPaddingSize + jsonDataSize + totalBinarySize);

    // double time3 =
    // std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();

    // std::cout << "Attention Make Time = " << (time1 - time0) << "(ms) ==>
    // Object Make Time = " << (time2 - time1) << "(ms) ==> Image & Point
    // RsWebsocketSerialize::make Time = " << (time3 - time2) << "(ms)" <<
    // std::endl;

    return _message;
}

// Serialize GLTF Frame to Buffer
int RsWebsocketSerialize::gltfSerialize(const xvizGLTF::Ptr &gltfPtr, std::vector<char> &serializeBuffer) {
    size_t gltfSize = gltfPtr->getGltfSize();
    if (serializeBuffer.size() < gltfSize) {
        serializeBuffer.resize(gltfSize);
    }

    int offset = 0;
    memcpy(serializeBuffer.data() + offset, gltfPtr->gltfHeader.data(),
           xvizGLTF::GLTF_OFFSET::GLTF_GLTF_HEADER_SIZE);
    offset += xvizGLTF::GLTF_OFFSET::GLTF_GLTF_HEADER_SIZE;

    memcpy(serializeBuffer.data() + offset, gltfPtr->jsonData.data(), gltfPtr->jsonData.size());
    offset += gltfPtr->jsonData.size();

    if (gltfPtr->jsonPadding.size() > 0) {
        memcpy(serializeBuffer.data() + offset, gltfPtr->jsonPadding.data(), gltfPtr->jsonPadding.size());
        offset += gltfPtr->jsonPadding.size();
    }

    memcpy(serializeBuffer.data() + offset, gltfPtr->binaryHeader.data(),
           xvizGLTF::GLTF_OFFSET::GLTF_BINARY_HEADER_SIZE);
    offset += xvizGLTF::GLTF_OFFSET::GLTF_BINARY_HEADER_SIZE;

    if (gltfPtr->binaryData.size() > 0) {
        memcpy(serializeBuffer.data() + offset, gltfPtr->binaryData.data(), gltfPtr->binaryData.size());
        offset += gltfPtr->binaryData.size();
    }

    return offset;
}

int RsWebsocketSerialize::loadFillMap() {
    size_t fillMapsCnt = _commConfig.fillMapsConfig.maps.size();
    _mapPointCntBuffers.clear();
    _mapColorBuffers.clear();
    _mapPointBuffers.clear();

    _mapPointCntBuffers.resize(fillMapsCnt, 0);
    _mapPointBuffers.resize(fillMapsCnt, std::vector<char>());
    _mapColorBuffers.resize(fillMapsCnt, std::vector<char>());
    for (size_t iterMap = 0; iterMap < fillMapsCnt; ++iterMap) {
        std::string mapPath = _commConfig.fillMapsConfig.maps[iterMap];
        std::string mapFormat = mapPath.substr(mapPath.size() - 4);

        if (mapFormat == ".pcd") {
            // Load & Filter
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
            int ret = pcl::io::loadPCDFile(mapPath, *cloudPtr);

            if (ret != 0) {
                std::cout << "PCL Load File Faild: " << mapPath << std::endl;
                return -1;
            }

            int totalPointCloudCnt = cloudPtr->size();
            int filterPointCnt = 0;
            int erasePointCnt = 0;
            for (int i = 0; i < totalPointCloudCnt - erasePointCnt; ++i) {
                const pcl::PointXYZI &point = cloudPtr->points[i];
                if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                    int swapIndex = totalPointCloudCnt - erasePointCnt - 1;
                    cloudPtr->points[i] = cloudPtr->points[swapIndex];
                    --i;
                    ++erasePointCnt;
                    continue;
                }
                else {
                    filterPointCnt++;
                }
            }
            cloudPtr->resize(filterPointCnt);

            // 更新总点数量
            totalPointCloudCnt = filterPointCnt;
            _mapPointCntBuffers[iterMap] = filterPointCnt;

            // 分配内存
            _mapPointBuffers[iterMap].resize(filterPointCnt * 12);
            _mapColorBuffers[iterMap].resize(filterPointCnt * 4);

            // colors:
            int binaryOffset = 0;
            if (!_commConfig.mapColorMapConfig.colorMapperEnable) {  // No Color Mapper
                for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                    memcpy(_mapColorBuffers[iterMap].data() + binaryOffset, _mapColors, sizeof(_mapColors));
                    binaryOffset += 4;
                }
            }
            else {  // Color Mapper
                int factorCnt = 0;
                std::vector<float> lows;
                std::vector<float> highs;
                std::vector<float> alphas;

                int factorEnableCnt = _commConfig.mapColorMapConfig.factorsEnable.size();

                for (int i = 0; i < factorEnableCnt; ++i) {
                    if (_commConfig.mapColorMapConfig.factorsEnable[i]) {
                        factorCnt += 1;
                        lows.push_back(_commConfig.mapColorMapConfig.factorsLow[i]);
                        highs.push_back(_commConfig.mapColorMapConfig.factorsUp[i]);
                        alphas.push_back(_commConfig.mapColorMapConfig.factorsAlphas[i]);
                    }
                }

                if (factorCnt == 1 &&
                _commConfig.mapColorMapConfig.factorsEnable[static_cast<int>(
                RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_INTENSITY)] &&
                _commConfig.colorMapConfig.colorMapperType == RS_COLOR_MAP_TYPE::RS_POINTCLOUD_COLOR_MAP_RVIZ_TYPE) {
                    std::vector<st_ColorMapRGB_UCHAR> mappedColors(totalPointCloudCnt);
                    for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                        mappedColors[iterPc].r = 255.0f;
                        mappedColors[iterPc].g = std::min(50.0f, cloudPtr->points[iterPc].intensity) / 50.0f * 255.0f;
                        mappedColors[iterPc].b = 0.0f;
                        mappedColors[iterPc].a = 255.0f;
                    }
                    int totalColorBinarySize = totalPointCloudCnt * 4;
                    memcpy(_mapColorBuffers[iterMap].data(), mappedColors.data(), totalColorBinarySize);
                }
                else if (factorCnt == 1) {
                    std::vector<float> srcArrays(totalPointCloudCnt, 0.0f);

                    if (_commConfig.mapColorMapConfig.factorsEnable[static_cast<int>(
                    RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_X)]) {
                        for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                            srcArrays[iterPc] = cloudPtr->points[iterPc].x;
                        }
                    }
                    else if (_commConfig.mapColorMapConfig.factorsEnable[static_cast<int>(
                    RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Y)]) {
                        for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                            srcArrays[iterPc] = cloudPtr->points[iterPc].y;
                        }
                    }
                    else if (_commConfig.mapColorMapConfig.factorsEnable[static_cast<int>(
                    RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Z)]) {
                        for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                            srcArrays[iterPc] = cloudPtr->points[iterPc].z;
                        }
                    }
                    else if (_commConfig.mapColorMapConfig.factorsEnable[static_cast<int>(
                    RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_INTENSITY)]) {
                        for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                            srcArrays[iterPc] = cloudPtr->points[iterPc].intensity;
                        }
                    }

                    RSColorMapper colorMapper(_commConfig.mapColorMapConfig.colorMapperType, 100.0,
                    (_commConfig.mapColorMapConfig.enableColorMapInvert ? false : true));

                    // Default st_ColorMapRGB_UCHAR.a = 255
                    std::vector<st_ColorMapRGB_UCHAR> mappedColors;
                    st_ColorMapRGB_UCHAR bottomColor;

                    colorMapper.m_applyColorMap(srcArrays, lows[0], highs[0],
                                                mappedColors, bottomColor);

                    int totalColorBinarySize = totalPointCloudCnt * 4;
                    memcpy(_mapColorBuffers[iterMap].data(), mappedColors.data(), totalColorBinarySize);
                }
                else if (factorCnt > 1) {
                    std::vector<std::vector<float>> srcArrays(factorCnt, std::vector<float>());

                    float totalAlphas = 0;
                    for (int i = 0; i < factorCnt; ++i) {
                        srcArrays[i].resize(totalPointCloudCnt, 0.0f);
                        totalAlphas += std::abs(alphas[i]);
                    }

                    for (int i = 0; i < factorCnt; ++i) {
                        alphas[i] = (totalAlphas < 0.01) ? (1.0f / factorCnt) : alphas[i] / totalAlphas;
                    }

                    for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                        int index = 0;

                        if (_commConfig.mapColorMapConfig.factorsEnable[static_cast<int>(
                        RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_X)]) {
                            srcArrays[index][iterPc] = cloudPtr->points[iterPc].x;
                            index += 1;
                        }

                        if (_commConfig.mapColorMapConfig.factorsEnable[static_cast<int>(
                        RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Y)]) {
                            srcArrays[index][iterPc] = cloudPtr->points[iterPc].y;
                            index += 1;
                        }

                        if (_commConfig.mapColorMapConfig.factorsEnable[static_cast<int>(
                        RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_Z)]) {
                            srcArrays[index][iterPc] = cloudPtr->points[iterPc].z;
                            index += 1;
                        }

                        if (_commConfig.mapColorMapConfig.factorsEnable[static_cast<int>(
                        RS_COLOR_MAP_FACTOR_INDEX::RS_COLOR_MAP_FACTOR_INTENSITY)]) {
                            srcArrays[index][iterPc] = cloudPtr->points[iterPc].intensity;
                            index += 1;
                        }
                    }

                    RSColorMapper colorMapper(_commConfig.mapColorMapConfig.colorMapperType, 100.0,
                    (_commConfig.mapColorMapConfig.enableColorMapInvert ? false : true));

                    // Default st_ColorMapRGB_UCHAR.a = 255
                    std::vector<st_ColorMapRGB_UCHAR> mappedColors;
                    st_ColorMapRGB_UCHAR bottomColor;

                    colorMapper.m_applyColorMap(srcArrays, alphas, lows, highs,mappedColors, bottomColor);

                    int totalColorBinarySize = totalPointCloudCnt * 4;
                    memcpy(_mapColorBuffers[iterMap].data(), mappedColors.data(), totalColorBinarySize);
                }
                else {
                    for (int iterPc = 0; iterPc < totalPointCloudCnt; ++iterPc) {
                        memcpy(_mapColorBuffers[iterMap].data(), _mapColors, sizeof(_mapColors));
                        binaryOffset += 4;
                    }
                }
            }
            // Points
            int binaryPointOffset = 0;
            for (int i = 0; i < totalPointCloudCnt; ++i) {
                memcpy(_mapPointBuffers[iterMap].data() + binaryPointOffset, &(cloudPtr->points[i].x), 12);
                binaryPointOffset += 12;
            }
            }
        else {
            RERROR << "Not Support Map Format";
            return -2;
        }
    }

    return 0;
}

std::string RsWebsocketSerialize::fromTypeIDToType(const int objTypeId) {
    std::string type = "unknown";
    int objectConfigCnt = _commConfig.objectConfigs.size();
    for (int j = 0; j < objectConfigCnt; ++j) {
        if (objTypeId == _commConfig.objectConfigs[j].typeID) {
            type = _commConfig.objectConfigs[j].type;
            break;
        }
    }

    return type;
}

static void multiMatrixVector2f(const RsMatrix2f &mat, const RsVector2f &vec, RsVector2f &res) {
    res.x = mat.val[0][0] * vec.x + mat.val[0][1] * vec.y;
    res.y = mat.val[1][0] * vec.x + mat.val[1][1] * vec.y;
}

int RsWebsocketSerialize::makeSerializeObjectCorners(const std::vector<robosense::perception::Object::Ptr> &objects,
                                                     std::vector<std::vector<RsXvizPoint3D>> &objectsCorners) {
    objectsCorners.clear();

    if (objects.size() == 0) {
        return 1;
    }

    objectsCorners.resize(objects.size());
    auto totalObjectCnt = objects.size();

    for (size_t iterObj = 0; iterObj < totalObjectCnt; ++iterObj) {
        const robosense::perception::Object::Ptr roboObj = objects[iterObj];

        float yaw = std::atan2(roboObj->core_infos_.direction.y,
        roboObj->core_infos_.direction.x);
        RsMatrix2f rotateM;
        rotateM.val[0][0] = std::cos(yaw);
        rotateM.val[0][1] = -std::sin(yaw);
        rotateM.val[1][0] = std::sin(yaw);
        rotateM.val[1][1] = std::cos(yaw);
        float halfLen = roboObj->core_infos_.size.x / 2.0;
        float halfWidth = roboObj->core_infos_.size.y / 2.0;

        RsVector2f corners1, corners2, corners3, corners4;
        RsVector2f newCorners1, newCorners2, newCorners3, newCorners4;
        corners1.x = -halfLen;
        corners1.y = -halfWidth;

        corners2.x = halfLen;
        corners2.y = -halfWidth;

        corners3.x = halfLen;
        corners3.y = halfWidth;

        corners4.x = -halfLen;
        corners4.y = halfWidth;

        // newCorners1 = rotateM * corners1;
        // newCorners2 = rotateM * corners2;
        // newCorners3 = rotateM * corners3;
        // newCorners4 = rotateM * corners4;
        multiMatrixVector2f(rotateM, corners1, newCorners1);
        multiMatrixVector2f(rotateM, corners2, newCorners2);
        multiMatrixVector2f(rotateM, corners3, newCorners3);
        multiMatrixVector2f(rotateM, corners4, newCorners4);

        RsVector3f translateXY(roboObj->core_infos_.center.x,
        roboObj->core_infos_.center.y, 0.0f);

        //            std::vector<QJsonArray> vertices(8, QJsonArray());
        std::vector<RsXvizPoint3D> vertices(8, RsXvizPoint3D());

        float minZ = roboObj->core_infos_.center.z - roboObj->core_infos_.size.z / 2.0;
        float maxZ = roboObj->core_infos_.center.z + roboObj->core_infos_.size.z / 2.0;
        for (int iterCorner = 0; iterCorner < 4; ++iterCorner) {
            if (iterCorner == 0) {
                // BottomLayer
                vertices[iterCorner].x = newCorners1.x + translateXY.x;
                vertices[iterCorner].y = newCorners1.y + translateXY.y;
                vertices[iterCorner].z = minZ;

                vertices[iterCorner].updateJsonArray();

                // TopLayer
                vertices[iterCorner + 4].x = newCorners1.x + translateXY.x;
                vertices[iterCorner + 4].y = newCorners1.y + translateXY.y;
                vertices[iterCorner + 4].z = maxZ;

                vertices[iterCorner + 4].updateJsonArray();
            }
            else if (iterCorner == 1) {
                // BottomLayer
                vertices[iterCorner].x = newCorners2.x + translateXY.x;
                vertices[iterCorner].y = newCorners2.y + translateXY.y;
                vertices[iterCorner].z = minZ;

                vertices[iterCorner].updateJsonArray();

                // TopLayer
                vertices[iterCorner + 4].x = newCorners2.x + translateXY.x;
                vertices[iterCorner + 4].y = newCorners2.y + translateXY.y;
                vertices[iterCorner + 4].z = maxZ;

                vertices[iterCorner + 4].updateJsonArray();
            }
            else if (iterCorner == 2) {
                // BottomLayer
                vertices[iterCorner].x = newCorners3.x + translateXY.x;
                vertices[iterCorner].y = newCorners3.y + translateXY.y;
                vertices[iterCorner].z = minZ;

                vertices[iterCorner].updateJsonArray();

                // TopLayer
                vertices[iterCorner + 4].x = newCorners3.x + translateXY.x;
                vertices[iterCorner + 4].y = newCorners3.y + translateXY.y;
                vertices[iterCorner + 4].z = maxZ;

                vertices[iterCorner + 4].updateJsonArray();
            }
            else if (iterCorner == 3) {
                // BottomLayer
                vertices[iterCorner].x = newCorners4.x + translateXY.x;
                vertices[iterCorner].y = newCorners4.y + translateXY.y;
                vertices[iterCorner].z = minZ;

                vertices[iterCorner].updateJsonArray();

                // TopLayer
                vertices[iterCorner + 4].x = newCorners4.x + translateXY.x;
                vertices[iterCorner + 4].y = newCorners4.y + translateXY.y;
                vertices[iterCorner + 4].z = maxZ;

                vertices[iterCorner + 4].updateJsonArray();
            }
        }
        objectsCorners[iterObj] = vertices;
    }

    return 0;
}

int RsWebsocketSerialize::makeSerializeObjectPolygons(const std::vector<robosense::perception::Object::Ptr> &objects,
                                                      std::vector<std::vector<RsXvizPoint3D>> &objectsPolygons) {
    objectsPolygons.clear();

    objectsPolygons.resize(objects.size());
    auto totalObjectCnt = objects.size();
    // for (int iterObj = 0; iterObj < objects.size(); ++iterObj)
    for (size_t iterObj = 0; iterObj < totalObjectCnt; ++iterObj) {
        const robosense::perception::Object::Ptr &roboObj = objects[iterObj];
        float minZ = roboObj->core_infos_.center.z - roboObj->core_infos_.size.z / 2.0;
        float maxZ = roboObj->core_infos_.center.z + roboObj->core_infos_.size.z / 2.0;

        // std::vector<QJsonArray> vertices;
        std::vector<RsXvizPoint3D> vertices;
        int polygonCnt = roboObj->supplement_infos_.polygon.size();

        vertices.resize(polygonCnt * 2, RsXvizPoint3D());
        for (int iterPolygon = 0; iterPolygon < polygonCnt; ++iterPolygon) {
            const RsVector3f &polygonPoint = roboObj->supplement_infos_.polygon[iterPolygon];

            // BottomLayer
            vertices[iterPolygon].x = polygonPoint.x;
            vertices[iterPolygon].y = polygonPoint.y;
            vertices[iterPolygon].z = minZ;

            vertices[iterPolygon].updateJsonArray();

            // TopLayer
            vertices[iterPolygon + polygonCnt].x = polygonPoint.x;
            vertices[iterPolygon + polygonCnt].y = polygonPoint.y;
            vertices[iterPolygon + polygonCnt].z = maxZ;

            vertices[iterPolygon + polygonCnt].updateJsonArray();
        }

        objectsPolygons[iterObj] = vertices;
    }
    return 0;
}

int RsWebsocketSerialize::makeSerializeAttentionObjectPolygons(
const std::vector<robosense::perception::Object::Ptr> &objects,
std::vector<std::vector<RsXvizPoint3D>> &objectsPolygons) {
    objectsPolygons.clear();

    objectsPolygons.resize(objects.size());

    int totalObjectCnt = objects.size();
    // for (int iterObj = 0; iterObj < objects.size(); ++iterObj)
    for (int iterObj = 0; iterObj < totalObjectCnt; ++iterObj) {
        const robosense::perception::Object::Ptr &roboObj = objects[iterObj];
        float minZ = roboObj->core_infos_.center.z - roboObj->core_infos_.size.z / 2.0;
        float maxZ = roboObj->core_infos_.center.z + roboObj->core_infos_.size.z / 2.0;

        int polygonCnt = roboObj->supplement_infos_.polygon.size();
        std::vector<RsXvizPoint3D> vertices(2 * polygonCnt, RsXvizPoint3D());
        for (int iterRef = 0; iterRef < polygonCnt; ++iterRef) {
            // const Eigen::Vector3f &referencePoint =
            // roboObj->supplement_infos_.reference[iterRef];
            const RsVector3f &referencePoint = roboObj->supplement_infos_.polygon[iterRef];

            // BottomLayer
            vertices[iterRef].x = referencePoint.x;
            vertices[iterRef].y = referencePoint.y;
            vertices[iterRef].z = minZ;

            vertices[iterRef].updateJsonArray();

            // TopLayer
            vertices[iterRef + polygonCnt].x = referencePoint.x;
            vertices[iterRef + polygonCnt].y = referencePoint.y;
            vertices[iterRef + polygonCnt].z = maxZ;

            vertices[iterRef + polygonCnt].updateJsonArray();
        }
        objectsPolygons[iterObj] = vertices;
    }

    return 0;
}

// "vertices":[[x_1, y_1, z_1], ... , [x_n, y_n, z_n]]
int RsWebsocketSerialize::makeSerializeCubeBox(const std::vector<RsXvizPoint3D> &cornerPolygons, std::string &vertices){
    int cornerPolygonsCnt = cornerPolygons.size();
    vertices.clear();

    if (cornerPolygonsCnt <= 2) {
        vertices = "\"vertices\":[]";
        return 1;
    }

    vertices += "\"vertices\":[";
    int halfCornerPolygonsCnt = cornerPolygonsCnt / 2;
    for (int i = 0; i <= halfCornerPolygonsCnt; ++i) {
        if (i != halfCornerPolygonsCnt) {
            vertices += "[" + std::to_string(cornerPolygons[i].x) + "," +
            std::to_string(cornerPolygons[i].y) + "," +
            std::to_string(cornerPolygons[i].z) + "],";
        }
        else {
            vertices += "[" + std::to_string(cornerPolygons[0].x) + "," +
            std::to_string(cornerPolygons[0].y) + "," +
            std::to_string(cornerPolygons[0].z) + "]";
        }
    }
    vertices += "]";
    return 0;
}

// "vertices":[[x_1,y_1,z_1],...,[x_n,y_n,z_n]]
int RsWebsocketSerialize::makeSerializeAttentionCubeBox(
const std::vector<RsXvizPoint3D> &cornerPolygons, std::string &vertices) {
    int cornerPolygonsCnt = cornerPolygons.size();
    vertices.clear();
    if (cornerPolygonsCnt <= 2) {
        vertices = "\"vertices\":[]";
        return 1;
    }

    // std::cout << __FUNCTION__ << "==> " << __LINE__ << ": " <<
    // cornerPolygonsCnt << std::endl;

    vertices += "\"vertices\":[";
    int halfCornerPolygonsCnt = cornerPolygonsCnt / 2;
    for (int i = 0; i < halfCornerPolygonsCnt; ++i) {
        vertices += "[" + std::to_string(cornerPolygons[i].x) + "," +
        std::to_string(cornerPolygons[i].y) + "," +
        std::to_string(cornerPolygons[i].z) + "],";
    }

    for (int i = halfCornerPolygonsCnt - 2; i >= 0; --i) {
        if (i != 0) {
            vertices += "[" + std::to_string(cornerPolygons[i].x) + "," +
            std::to_string(cornerPolygons[i].y) + "," +
            std::to_string(cornerPolygons[i].z) + "],";
        }
        else {
            vertices += "[" + std::to_string(cornerPolygons[i].x) + "," +
            std::to_string(cornerPolygons[i].y) + "," +
            std::to_string(cornerPolygons[i].z) + "]";
        }
    }
    vertices += "]";

    return 0;
}

// "vertices":[[x_1, y_1, z_1],...,[x_n, y_n, z_n]]
int RsWebsocketSerialize::makeSerializeGridBox(const std::vector<RsXvizPoint3D> &cornerPolygons, std::string &vertices) {
    auto cornerPolygonCnt = cornerPolygons.size();
    vertices.clear();
    if (cornerPolygonCnt <= 2) {
        vertices = "\"vertices\":[]";
        return 1;
    }

    vertices = "\"vertices\":[";

    auto halfObjectPolygonsCnt = cornerPolygonCnt / 2;

    // Bottom Layer Points
    for (size_t i = 0; i <= halfObjectPolygonsCnt; ++i) {
        vertices += cornerPolygons[i % halfObjectPolygonsCnt].json_array + ",";
    }

    // Top Layer First Point
    vertices += cornerPolygons[halfObjectPolygonsCnt].json_array + ",";

    // Top Layer Points
    for (size_t i = halfObjectPolygonsCnt + 1; i < cornerPolygonCnt; ++i) {
        vertices += cornerPolygons[i].json_array + "," +
        cornerPolygons[i - halfObjectPolygonsCnt].json_array + "," +
        cornerPolygons[i].json_array + ",";
    }

    // Top Layer First Point
    vertices += cornerPolygons[halfObjectPolygonsCnt].json_array;

    vertices += "]";

    return 0;
}

// "vertices":[[x_1, y_1, z_1],...,[x_n, y_n, z_n]]
int RsWebsocketSerialize::makeSerializeAttentionGridBox(
const std::vector<RsXvizPoint3D> &cornerPolygons, std::string &vertices) {
    int cornerPolygonsCnt = cornerPolygons.size();

    vertices.clear();
    if (cornerPolygonsCnt <= 2) {
        vertices = "\"vertices\":[]";
        return 1;
    }

    vertices = "\"vertices\":[";

    int halfCornerPolygonCnt = cornerPolygonsCnt / 2;

    for (int i = 0; i < halfCornerPolygonCnt - 1; ++i) {
        vertices += cornerPolygons[i].json_array + "," +
        cornerPolygons[(i + 1)].json_array + ",";
    }

    vertices += cornerPolygons[halfCornerPolygonCnt - 1].json_array + "," +
    cornerPolygons[cornerPolygonsCnt - 1].json_array + ",";

    for (int i = cornerPolygonsCnt - 1; i > halfCornerPolygonCnt; --i) {
        vertices += cornerPolygons[i].json_array + "," +
        cornerPolygons[(i - 1)].json_array + ",";
    }

    vertices += cornerPolygons[halfCornerPolygonCnt].json_array + "," + cornerPolygons[0].json_array;

    vertices += "]";

    return 0;
}

// lane stream
int RsWebsocketSerialize::makeSerializeLanes(const std::vector<robosense::perception::Lane::Ptr> &lanes,
                                             const std::string &stream_id, const std::string &type,
                                             std::string &vertices) {
    vertices.clear();

    int totalLaneCnt = lanes.size();

    if (totalLaneCnt == 0) {
        vertices = "\"" + stream_id + "\":{\"" + type +
        "\":[{\"base\":{\"object_id\":\"lanes\"},\"vertices\":[]}]}";
        return 1;
    }

    vertices = "\"" + stream_id + "\":{\"" + type + "\":[";

    std::string json_array;
    int totalLaneCnt1 = totalLaneCnt - 1;
    for (int i = 0; i < totalLaneCnt; ++i) {
        json_array.clear();

        const robosense::perception::Lane::Ptr &roboLane = lanes[i];
        const robosense::perception::Curve &roboCurve = roboLane->curve;

        json_array = "{\"base\":{\"object_id\":\"lanes_" + std::to_string(i) + "\"},\"vertices\":[";

        float x_start = 0.0;
        float y_start = roboCurve.a * std::pow(x_start, 3) +
        roboCurve.b * std::pow(x_start, 2) + roboCurve.c * x_start +
        roboCurve.d;
        float x_end = 20.0;
        float y_end = roboCurve.a * std::pow(x_end, 3) +
        roboCurve.b * std::pow(x_end, 2) + roboCurve.c * x_end +
        roboCurve.d;

        json_array += std::string("[") + std::to_string(x_start) + "," +
        std::to_string(y_start) + "," + std::to_string(0.12) + "],[" +
        std::to_string(x_end) + "," + std::to_string(y_end) + "," +
        std::to_string(0.12) + "]]";

        if (i != totalLaneCnt1) {
            vertices += json_array + "},";
        }
        else {
            vertices += json_array + "}";
        }
    }
    vertices += "]}";

    return 0;
}

// curb stream
int RsWebsocketSerialize::makeSerializeCurbs(const std::vector<robosense::perception::Roadedge::Ptr> &curbs,
                                             const std::string &stream_id, const std::string &type,
                                             std::string &vertices) {
    vertices.clear();

    int totalCurbCnt = curbs.size();

    if (totalCurbCnt == 0) {
        vertices = "\"" + stream_id + "\":{\"" + type + "\":[{\"base\":{\"object_id\":\"curbs\"},\"vertices\":[]}]}";
        return 1;
    }

    vertices = "\"" + stream_id + "\":{\"" + type + "\":[";

    std::string json_array;
    int totalCurbCnt1 = totalCurbCnt - 1;
    for (int i = 0; i < totalCurbCnt; ++i) {
        json_array.clear();

        const robosense::perception::Roadedge::Ptr &roboCurb = curbs[i];
        const robosense::perception::Curve &roboCurve = roboCurb->curve;

        json_array = "{\"base\":{\"object_id\":\"lanes_" + std::to_string(i) + "\"},\"vertices\":[";

        float x_start = 0.0;
        float y_start = roboCurve.a * std::pow(x_start, 3) +
        roboCurve.b * std::pow(x_start, 2) + roboCurve.c * x_start +
        roboCurve.d;
        float x_end = 20.0;
        float y_end = roboCurve.a * std::pow(x_end, 3) +
        roboCurve.b * std::pow(x_end, 2) + roboCurve.c * x_end +
        roboCurve.d;

        json_array += std::string("[") + std::to_string(x_start) + "," +
        std::to_string(y_start) + "," + std::to_string(0.12) + "],[" +
        std::to_string(x_end) + "," + std::to_string(y_end) + "," +
        std::to_string(0.12) + "]]";

        if (i != totalCurbCnt1) {
            vertices += json_array + "},";
        }
        else {
            vertices += json_array + "}";
        }
    }

    vertices += "]}";

    return 0;
}

// ROI Stream
int RsWebsocketSerialize::makeSerializeRoi(const RsXvizRoiInfo &roiInfo, const std::string &stream_id,
                                           const std::string &type,const std::string &object_id,
                                           std::string &vertices) {
    vertices.clear();

    vertices = "\"" + stream_id + "\":{\"" + type + "\":[{\"base\":{\"object_id\":\"" + object_id +
    "\",\"style\":{\"height\":0.01}},\"vertices\":[";

    size_t anchorsCnt = roiInfo.anchors.size();

    // std::cout << "anchorsCnt = " << anchorsCnt << std::endl;

    for (size_t i = 0; i <= anchorsCnt; ++i) {
        // std::cout << "i = " << (i % anchorsCnt) << std::endl;

        // const Eigen::Vector3f &anchor = roiInfo.anchors[i % anchorsCnt];
        const RsVector3f &anchor = roiInfo.anchors[i % anchorsCnt];

        if (i != anchorsCnt) {
            vertices += "[" + std::to_string(anchor.x) + "," +
            std::to_string(anchor.y) + "," + std::to_string(0.03) + "], ";
        }
        else {
            vertices += "[" + std::to_string(anchor.x) + "," +
            std::to_string(anchor.y) + "," + std::to_string(0.03) + "]";
        }
    }
    vertices += "]}]}";

    return 0;
}

// freespace stream
int RsWebsocketSerialize::makeSerializeFreespace(const std::vector<RsVector3f> &freespaces,
                                                 const std::string &stream_id, const std::string &type,
                                                 std::string &vertices) {
    vertices.clear();

    int totalFreespacesCnt = freespaces.size();
    if (totalFreespacesCnt < 2) {
        vertices = "\"" + stream_id + "\":{\"" + type + "\":[{\"base\":{\"object_id\":\"freespace\",\"style\":{"
        "\"height\":0.01}},\"vertices\":[]}]}";
        return 1;
    }

    vertices = "\"" + stream_id + "\":{\"" + type + "\":[{\"base\":{\"object_id\":\"freespace\",\"style\":{\"height\":"
    "0.01}},\"vertices\":[";

    const auto &firstInfo = freespaces[0];
    const auto &lastInfo = freespaces[totalFreespacesCnt - 1];

    float firstYawAngle = std::atan2(firstInfo.y, firstInfo.x);
    float lastYawAngle = std::atan2(lastInfo.y, lastInfo.x);

    // std::cout << "firstInfo = " << firstInfo->yaw_angle << ", lastInfo = " <<
    // lastInfo->yaw_angle << std::endl;
    const float splitFreespace = 45.f / 180.0f * M_PI;
    if (std::abs(lastYawAngle - (firstYawAngle + 2 * M_PI)) < splitFreespace ||
    std::abs((lastYawAngle + 2 * M_PI) - firstYawAngle) < splitFreespace) {
        std::string json_array;
        for (int iterFp = 0; iterFp <= totalFreespacesCnt; ++iterFp) {
            const auto &roboFp = freespaces[iterFp % totalFreespacesCnt];

            json_array.clear();

            if (iterFp != totalFreespacesCnt) {
                json_array = std::string("[") + std::to_string(roboFp.x) + "," +
                std::to_string(roboFp.y) + "," + std::to_string(0.03) +
                "],";
            }
            else {
                json_array = std::string("[") + std::to_string(roboFp.x) + "," +
                std::to_string(roboFp.y) + "," + std::to_string(0.03) +
                "]";
            }
            vertices += json_array;
        }
    }
    else {
        std::string json_array;
        int totalFreespacesCnt1 = totalFreespacesCnt - 1;
        for (int iterFp = 0; iterFp < totalFreespacesCnt; ++iterFp) {
            const auto &roboFp = freespaces[iterFp];

            json_array.clear();

            if (iterFp != totalFreespacesCnt1) {
                json_array = std::string("[") + std::to_string(roboFp.x) + "," +
                std::to_string(roboFp.y) + "," + std::to_string(0.03) +
                "],";
            }
            else {
                json_array = std::string("[") + std::to_string(roboFp.x) + "," +
                std::to_string(roboFp.y) + "," + std::to_string(0.03) +
                "]";
            }
            vertices += json_array;
        }
    }
    vertices += "]}]}";
    return 0;
}

int RsWebsocketSerialize::makeSerializeTrajectory(const std::deque<RsVector3f> &trajectory,
                                                  std::vector<std::string> &vertices) {
    vertices.clear();

    auto trajecorySize = trajectory.size();
    if (trajecorySize == 0) {
        return 1;
    }

    for (size_t i = 0; i < trajecorySize; ++i) {
        const RsVector3f &trajPoint = trajectory[i];
        std::string json_array = "\"center\":[" + std::to_string(trajPoint.x) + ","
        + std::to_string(trajPoint.y) + "," + std::to_string(trajPoint.z) + "]";
        vertices.push_back(json_array);
    }
    return 0;
}

int RsWebsocketSerialize::makeSerializeBufferViews(const std::vector<std::pair<int, int>> &bufferInfos,
                                                   std::string &bufferViews) {
    bufferViews.clear();

    bufferViews = "\"bufferViews\":[";

    std::string json_array;
    int bufferInfoCnt = bufferInfos.size();
    int bufferInfoCnt1 = bufferInfoCnt - 1;
    for (int i = 0; i < bufferInfoCnt; ++i) {
        if (i != bufferInfoCnt1) {
            json_array = "{\"buffer\":0,\"byteLength\":" + std::to_string(bufferInfos[i].first) +
            ",\"byteOffset\":" + std::to_string(bufferInfos[i].second) + "},";
        }
        else {
            json_array = "{\"buffer\":0,\"byteLength\":" +
            std::to_string(bufferInfos[i].first) +
            ",\"byteOffset\":" + std::to_string(bufferInfos[i].second) +
            "}";
        }
        bufferViews += json_array;
    }
    bufferViews += "]";

    return 0;
}

// Point Accessors: accessorInfos[0]: colors, accessorInfos[1]: points
int RsWebsocketSerialize::makeSerializeAccessors(const std::vector<int> &accessorInfos, std::string &accessors) {
    accessors.clear();

    accessors = "\"accessors\":[";

    int accessorInfoCnt = accessorInfos.size();
    int accessorInfoCnt1 = accessorInfoCnt - 1;
    std::string json_array;
    for (int i = 0; i < accessorInfoCnt; ++i) {
        if (i != accessorInfoCnt1 && i % 2 == 0) {
            json_array = "{\"bufferView\":" + std::to_string(i) + ",\"componentType\":5121,\"count\":" +
            std::to_string(accessorInfos[i]) + ",\"type\":\"VEC4\"},";
        }
        else if (i != accessorInfoCnt1 && i % 2 == 1) {
            json_array = "{\"bufferView\":" + std::to_string(i) + ",\"componentType\":5126,\"count\":" +
            std::to_string(accessorInfos[i]) + ",\"type\":\"VEC3\"},";
        }
        else {
            json_array = "{\"bufferView\":" + std::to_string(i) + ",\"componentType\":5126,\"count\":" +
            std::to_string(accessorInfos[i]) + ",\"type\":\"VEC3\"}";
        }
        accessors += json_array;
    }
    accessors += "]";
    return 0;
}

int RsWebsocketSerialize::makeSerializeBuffers(const int byteLength, std::string &buffers) {
    buffers.clear();

    buffers = "\"buffers\":[{\"byteLength\":" + std::to_string(byteLength) + "}]";

    return 0;
}

int RsWebsocketSerialize::makeSerializeImageBuffers(const int bufferViewOffset, const int imageBufferViewCnt,
                                                    const int imageWidth, const int imageHeight, std::string &images) {
    images.clear();

    images = "\"images\":[";
    int imageBufferViewCnt1 = imageBufferViewCnt - 1;
    int viewOffset = bufferViewOffset;
    for (int i = 0; i < imageBufferViewCnt; ++i) {
        if (i != imageBufferViewCnt1) {
            images += "{\"bufferView\":" + std::to_string(viewOffset) + ",\"height\":" + std::to_string(imageHeight) +
            ",\"mimeType\":\"image/png\",\"width\":" + std::to_string(imageWidth) + "},";
        }
        else {
            images += "{\"bufferView\":" + std::to_string(viewOffset) + ",\"height\":" + std::to_string(imageHeight) +
            ",\"mimeType\":\"image/png\",\"width\":" + std::to_string(imageWidth) + "}";
        }
        viewOffset += 1;
    }
    images += "]";

    return 0;
}

int RsWebsocketSerialize::makeSerializeMesh(std::string &mesh) {
    mesh = "\"meshes\":[]";
    return 0;
}

// Make Pose Stream
int RsWebsocketSerialize::makeSerializePoseStream(const std::string &stream_id, const double timestamp,
                                                  const double map_ori_x, const double map_ori_y, const double map_ori_z,
                                                  const double pos_x, const double pos_y, const double pos_z,
                                                  const double ori_x, const double ori_y, const double ori_z,
                                                  std::string &pose) {
    pose = "\"poses\":{\"" + stream_id +
    "\":{\"mapOrigin\":{\"longitude\":" + std::to_string(map_ori_x) +
    ",\"latitude\":" + std::to_string(map_ori_y) +
    ",\"altitude\":" + std::to_string(map_ori_z) + "},\"position\":[" +
    std::to_string(pos_x) + "," + std::to_string(pos_y) + "," +
    std::to_string(pos_z) + "],\"orientation\":[" + std::to_string(ori_x) +
    "," + std::to_string(ori_y) + "," + std::to_string(ori_z) +
    "],\"timestamp\":" + std::to_string(timestamp) + "}}";

    return 0;
}

// Make Point Stream
int RsWebsocketSerialize::makeSerializePointStream(const std::string &streamId, std::string &pointStream,
                                                   const std::string &object_id, const int colorAccessor,
                                                   const int pointAccessor) {
    pointStream = "\"" + streamId +
                "\":{\"points\":[{\"base\":{\"object_id\":\"" + object_id +
                "\"},\"colors\":\"#/accessors/" +
                std::to_string(colorAccessor) + "\",\"points\":\"#/accessors/" +
                std::to_string(pointAccessor) + "\"}]}";

    return 0;
}

// Make Map Stream
int RsWebsocketSerialize::makeSerializeMapStream(const std::string &streamId, std::string &mapStream,
                                                 const std::string &object_id, const int colorAccessor,
                                                 const int pointAccessor) {
    mapStream = "\"" + streamId + "\":{\"points\":[{\"base\":{\"object_id\":\"" +
              object_id + "\"},\"colors\":\"#/accessors/" +
              std::to_string(colorAccessor) + "\",\"points\":\"#/accessors/" +
              std::to_string(pointAccessor) + "\"}]}";
    return 0;
}

}  // namespace perception
}  // namespace robosense

#endif  // ROBOSENSE_WEBSOCKET_FOUND