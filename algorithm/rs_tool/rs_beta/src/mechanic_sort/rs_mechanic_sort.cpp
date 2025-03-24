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

#include <algorithm>
#include <limits>
#include <numeric>
#include "rs_beta/mechanic_sort/rs_mechanic_sort.h"
#include "rs_common/external/rs_define.h"

namespace robosense {

void RsMechanicSort::sort(const RsPointCloudGPT::Ptr &in_cloud_ptr, std::vector<int> &sort_2_in_indice) {
    if (in_cloud_ptr == nullptr) {
        RERROR << name() << ": input cloud is null!";
        RS_THROW("null ptr!");
    }

    sort_2_in_indice.clear();
    sort_2_in_indice.resize(in_cloud_ptr->size());
    if (in_cloud_ptr->height <= 1 || in_cloud_ptr->width <= 1) {
        std::iota(sort_2_in_indice.begin(), sort_2_in_indice.end(), 0);
        RDEBUG << name() << ": in cloud is not a sort!";
        return;
    }

    const auto height = in_cloud_ptr->height;
    const auto width = in_cloud_ptr->width;

    if (!init_) {
        angle_vec_.resize(height);
        for (size_t i = 0; i < height; ++i) {
            angle_vec_[i].first = -1;
            angle_vec_[i].second = std::numeric_limits<float>::min();
        }

        for (size_t row = 0; row < height; ++row) {
            int idx = -1;
            for (size_t col = 0; col < width; ++col) {
                int pt_idx = col * height + row;
                const auto &pt = in_cloud_ptr->points[pt_idx];
                if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                    continue;
                }
                if (std::abs(pt.x) < 1.e-3 || std::abs(pt.y) < 1.e-3) {
                    continue;
                }
                idx = col;
                break;
            }
            if (idx < 0) {
                continue;
            }
            std::pair<int , float> tmp_pair;
            tmp_pair.first = row;
            int pt_idx = idx * height + row;
            const auto &pt = in_cloud_ptr->points[pt_idx];
            float tmp_range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            tmp_pair.second = std::atan2(pt.z, tmp_range);
            angle_vec_[row] = tmp_pair;
        }

        std::sort(angle_vec_.begin(), angle_vec_.end(),
                  [](const std::pair<int, float> &p1, const std::pair<int, float> &p2) {
                      return p1.second < p2.second;
                  });

        init_ = true;
        // if some line all nan,set init false for next to init
        for (size_t i = 0; i < angle_vec_.size(); ++i) {
            if (angle_vec_[i].first < 0) {
                init_ = false;
            }
        }
    }

    std::vector<std::pair<int, int> > start_idx_pair(height);
    float hori_res = RS_M_PI * 2. / static_cast<float>(width);


    for (size_t row = 0; row < height; ++row) {
        int idx = -1;
        for (size_t col = 0; col < width; ++col) {
            int pt_idx = col * height + row;
            const auto &pt = in_cloud_ptr->points[pt_idx];
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                continue;
            }
            if (std::abs(pt.x) < 1.e-3 || std::abs(pt.y) < 1.e-3) {
                continue;
            }
            idx = col;
            break;
        }
        if (idx < 0) {
            continue;
        }
        int pt_idx = idx * height + row;
        const auto &pt = in_cloud_ptr->points[pt_idx];
        float angle = std::atan2(pt.y, pt.x) + RS_M_PI;
        int start_idx = angle / hori_res;
        std::pair<int, int> tmp_start_idx_pair;
        tmp_start_idx_pair.first = start_idx;
        tmp_start_idx_pair.second = idx;
        start_idx_pair[row] = tmp_start_idx_pair;
    }

    for (size_t row = 0; row < height; ++row) {
        int row_idx = angle_vec_[row].first;
        if (row_idx < 0) {
            continue;
        }
        const auto &tmp_pair = start_idx_pair[row_idx];
        for (size_t col = 0; col < width; ++col) {
            int ori_col = col + tmp_pair.second + tmp_pair.first;

            while (ori_col >= static_cast<int>(width)) {
                ori_col -= width;
            }
            while (ori_col < 0) {
                ori_col += width;
            }

            int ori_idx = ori_col * height + row_idx;
            int idx = row * width + col;
            sort_2_in_indice[idx] = ori_idx;
        }
    }
}

}   // namespace robosense
