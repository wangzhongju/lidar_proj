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
#include "rs_preprocessing/sender/external/cloud_sender.h"

namespace robosense {
namespace preprocessing {

void CloudSender::init(const RsYamlNode &config_node, const RsPreprocessing::Ptr &preprocessing) {
    RsYamlNode cloud_sender_node = config_node;
    impl_vec_.clear();
    std::string method;
    for (size_t i = 0; i < cloud_sender_node.size(); ++i) {
        rsYamlRead(cloud_sender_node[i], "method", method);
        std::string strategy = method + "CloudSender";
        if (!BaseCloudSenderRegisterer::IsValid(strategy)) {
            RERROR << name() << ": init strategy " << strategy << " failed!";
            RS_THROW("init register class error!");
        }
        BaseCloudSender::Ptr impl_ptr(BaseCloudSenderRegisterer::getInstanceByName(strategy));
        impl_ptr->init(cloud_sender_node[i], preprocessing);
        impl_vec_.push_back(impl_ptr);
    }
}

void CloudSender::send(const lidar::LidarPointCloudMsg::Ptr& lidar_msg) {
    for (auto impl_ptr : impl_vec_) {
        if (impl_ptr != nullptr) {
            impl_ptr->send(lidar_msg);
        }
    }
}

void CloudSender::addData(const lidar::LidarPointCloudMsg::Ptr &lidar_msg) {
    for (auto impl_ptr : impl_vec_) {
        if (impl_ptr != nullptr) {
            impl_ptr->addData(lidar_msg);
        }
    }
}
void CloudSender::start() {
    for (auto impl_ptr : impl_vec_) {
        if (impl_ptr != nullptr) {
            impl_ptr->start();
        }
    }
}

void CloudSender::stop() {
    for (auto impl_ptr : impl_vec_) {
        if (impl_ptr != nullptr) {
            impl_ptr->stop();
        }
    }
}

}   // namespace preprocessing
}   // namespace robosense