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

#ifndef RS_RVIZ_DISPLAY_EXTERNAL_COMMON_UTILS_H_
#define RS_RVIZ_DISPLAY_EXTERNAL_COMMON_UTILS_H_

#include "rs_common/external/common.h"
#include "rs_dependence/rs_dependence_manager.h"

#ifdef RS_ROS_FOUND

namespace robosense {
namespace perception {

inline void transRsMatToCvMat(const RsMat<RS8UC3> &rs_mat, cv::Mat &cv_mat) {
    cv_mat = cv::Mat::zeros(rs_mat.rows, rs_mat.cols, CV_8UC3);
    mempcpy(cv_mat.data, rs_mat.data.data(), rs_mat.rows * rs_mat.cols * sizeof(RS8UC3));
}

inline void transRsMatToCvMat(const RsMat<RS8UC1> &rs_mat, cv::Mat &cv_mat) {
    cv_mat = cv::Mat::zeros(rs_mat.rows, rs_mat.cols, CV_8UC1);
    mempcpy(cv_mat.data, rs_mat.data.data(), rs_mat.rows * rs_mat.cols * sizeof(RS8UC1));
}

inline void transRsMatToCvMat(const RsMat<RS32F> &rs_mat, cv::Mat &cv_mat) {
    cv_mat = cv::Mat::zeros(rs_mat.rows, rs_mat.cols, CV_32F);
    mempcpy(cv_mat.data, rs_mat.data.data(), rs_mat.rows * rs_mat.cols * sizeof(RS32F));
}

inline void transRsMatToCvMat(const RsMat<RS32S> &rs_mat, cv::Mat &cv_mat) {
    cv_mat = cv::Mat::zeros(rs_mat.rows, rs_mat.cols, CV_32S);
    mempcpy(cv_mat.data, rs_mat.data.data(), rs_mat.rows * rs_mat.cols * sizeof(RS32S));
}

inline void transCvMatToRsMat(const cv::Mat &cv_mat, RsMat<RS8UC3> &rs_mat) {
    if (cv_mat.type() != CV_8UC3) {
        RERROR << "transCvMatToRsMat: Can not trans cv_mat to RS8UC3 rs_mat!";
        RS_THROW("trans error!");
    }
    rs_mat.data.resize(cv_mat.rows * cv_mat.cols);
    rs_mat.rows = cv_mat.rows;
    rs_mat.cols = cv_mat.cols;
    mempcpy(rs_mat.data.data(), cv_mat.data, cv_mat.rows * cv_mat.cols * sizeof(RS8UC3));
}

inline void transCvMatToRsMat(const cv::Mat &cv_mat, RsMat<RS8UC1> &rs_mat) {
    if (cv_mat.type() != CV_8UC1) {
        RERROR << "transCvMatToRsMat: Can not trans cv_mat to RS8UC1 rs_mat!";
        RS_THROW("trans error!");
    }
    rs_mat.data.resize(cv_mat.rows * cv_mat.cols);
    rs_mat.rows = cv_mat.rows;
    rs_mat.cols = cv_mat.cols;
    mempcpy(rs_mat.data.data(), cv_mat.data, cv_mat.rows * cv_mat.cols * sizeof(RS8UC1));
}

inline void transCvMatToRsMat(const cv::Mat &cv_mat, RsMat<RS32F> &rs_mat) {
    if (cv_mat.type() != CV_32F) {
        RERROR << "transCvMatToRsMat: Can not trans cv_mat to RS32F rs_mat!";
        RS_THROW("trans error!");
    }
    rs_mat.data.resize(cv_mat.rows * cv_mat.cols);
    rs_mat.rows = cv_mat.rows;
    rs_mat.cols = cv_mat.cols;
    mempcpy(rs_mat.data.data(), cv_mat.data, cv_mat.rows * cv_mat.cols * sizeof(RS32F));
}

inline void transCvMatToRsMat(const cv::Mat &cv_mat, RsMat<RS32S> &rs_mat) {
    if (cv_mat.type() != CV_32S) {
        RERROR << "transCvMatToRsMat: Can not trans cv_mat to RS32S rs_mat!";
        RS_THROW("trans error!");
    }
    rs_mat.data.resize(cv_mat.rows * cv_mat.cols);
    rs_mat.rows = cv_mat.rows;
    rs_mat.cols = cv_mat.cols;
    mempcpy(rs_mat.data.data(), cv_mat.data, cv_mat.rows * cv_mat.cols * sizeof(RS32S));
}

inline void transRsCloudToPclCloud(const RsPointCloudGPT::Ptr &scan_ptr,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ptr) {
    if (scan_ptr == nullptr) {
        RERROR << "transRsCloudToPclCloud: input is null ptr!";
        RS_THROW("null ptr!");
    }
    if (cloud_ptr == nullptr) {
        cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
    cloud_ptr->height = scan_ptr->height;
    cloud_ptr->width = scan_ptr->width;
    cloud_ptr->points.resize(scan_ptr->points.size());
    if (g_point_type == PointType::POINTXYZI) {
        memcpy(cloud_ptr->points.data(), scan_ptr->points.data(),
               scan_ptr->points.size() * sizeof(RsPointXYZI));
    } else {
        for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
            auto &pt = cloud_ptr->points[i];
            const auto &scan_pt = scan_ptr->points[i];
            pt.x = scan_pt.x;
            pt.y = scan_pt.y;
            pt.z = scan_pt.z;
            pt.intensity = scan_pt.intensity;
        }
    }
}

inline void transPcCloudToRslCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr,
                                   RsPointCloudGPT::Ptr &scan_ptr) {
    if (cloud_ptr == nullptr) {
        RERROR << "transRcCloudToRslCloud: input is null ptr!";
        RS_THROW("null ptr!");
    }
    if (scan_ptr == nullptr) {
        scan_ptr.reset(new RsPointCloud<RsPoint>);
    }
    scan_ptr->height = cloud_ptr->height;
    scan_ptr->width = cloud_ptr->width;
    scan_ptr->points.resize(cloud_ptr->points.size());
    if (g_point_type == PointType::POINTXYZI) {
        memcpy(scan_ptr->points.data(), cloud_ptr->points.data(),
               cloud_ptr->points.size() * sizeof(pcl::PointXYZI));
    } else {
        for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
            const auto &pt = cloud_ptr->points[i];
            auto &scan_pt = scan_ptr->points[i];
            scan_pt.x = pt.x;
            scan_pt.y = pt.y;
            scan_pt.z = pt.z;
            scan_pt.intensity = pt.intensity;
        }
    }
}

inline void transPcCloudToRslCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud_ptr,
                                   RsPointCloudGPT &scan_ptr) {
    scan_ptr.height = cloud_ptr.height;
    scan_ptr.width = cloud_ptr.width;
    scan_ptr.points.resize(cloud_ptr.points.size());
    if (g_point_type == PointType::POINTXYZI) {
        memcpy(scan_ptr.points.data(), cloud_ptr.points.data(),
               cloud_ptr.points.size() * sizeof(pcl::PointXYZI));
    } else {
        for (size_t i = 0; i < cloud_ptr.points.size(); ++i) {
            const auto &pt = cloud_ptr.points[i];
            auto &scan_pt = scan_ptr.points[i];
            scan_pt.x = pt.x;
            scan_pt.y = pt.y;
            scan_pt.z = pt.z;
            scan_pt.intensity = pt.intensity;
        }
    }
}

inline void transPcCloudToRslCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr,
                                   RsPointCloudGPT::Ptr &scan_ptr) {
    if (cloud_ptr == nullptr) {
        RERROR << "transRcCloudToRslCloud: input is null ptr!";
        RS_THROW("null ptr!");
    }
    if (scan_ptr == nullptr) {
        scan_ptr.reset(new RsPointCloud<RsPoint>);
    }
    scan_ptr->height = cloud_ptr->height;
    scan_ptr->width = cloud_ptr->width;
    scan_ptr->points.resize(cloud_ptr->points.size());
    if (g_point_type == PointType::POINTXYZI) {
        memcpy(scan_ptr->points.data(), cloud_ptr->points.data(),
               cloud_ptr->points.size() * sizeof(pcl::PointXYZI));
    } else {
        for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
            const auto &pt = cloud_ptr->points[i];
            auto &scan_pt = scan_ptr->points[i];
            scan_pt.x = pt.x;
            scan_pt.y = pt.y;
            scan_pt.z = pt.z;
            scan_pt.intensity = pt.intensity;
        }
    }
}

}  // namespace perception
}  // namespace robosense

#endif  // RS_ROS_FOUND

#endif  // RS_RVIZ_DISPLAY_EXTERNAL_COMMON_UTILS_H_
