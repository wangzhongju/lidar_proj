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

#ifndef RS_RVIZ_DISPLAY_EXTERNAL_COMMON_BASE_PUB_H_
#define RS_RVIZ_DISPLAY_EXTERNAL_COMMON_BASE_PUB_H_

#include <functional>
#include "rviz_display/external/common/basic_types.h"

namespace robosense {
namespace perception {

static const char _prefix[] = "prefix";
static const char _map[] = "map";
static const char _road[] = "road";
static const char _strategy[] = "strategy";
static const char _map_frame_id[] = "map_frame_id";
static const char _frame_id[] = "frame_id";
static const char _display_axis[] = "display_axis";

class BasePubOptions {
public:
    std::string pre_fix = "";
    std::string frame_id = "/base_link";
    std::string map_frame_id = "/map";
    DisplayMode mode = DisplayMode::EFFICIENT;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_RVIZ_DISPLAY_EXTERNAL_COMMON_BASE_PUB_H_
