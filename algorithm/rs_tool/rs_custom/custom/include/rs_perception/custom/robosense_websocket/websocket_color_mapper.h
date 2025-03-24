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

#ifndef RS_PERCEPTION_CUSTOM_ROBOSENSE_WEBSOCKET_WEBSOCKET_COLOR_MAPPER_H_
#define RS_PERCEPTION_CUSTOM_ROBOSENSE_WEBSOCKET_WEBSOCKET_COLOR_MAPPER_H_

#include "rs_perception/custom/common/base_custom_params.h"

namespace robosense {
namespace perception {

struct st_ColorMapRGB_FLOAT {
    float r;
    float g;
    float b;

    st_ColorMapRGB_FLOAT() {
        r = 0.0;
        g = 0.0;
        b = 0.0;
    }

    st_ColorMapRGB_FLOAT(float r, float g, float b) {
        this->r = r;
        this->g = g;
        this->b = b;
    }
};

struct st_ColorMapRGB_UCHAR {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char a;

    st_ColorMapRGB_UCHAR() {
        r = 0;
        g = 0;
        b = 0;
        a = 255;
    }

    st_ColorMapRGB_UCHAR(unsigned char r, unsigned char g, unsigned char b) {
        this->r = r;
        this->g = g;
        this->b = b;
        this->a = 255;
    }
};

class RSColorMapper {
public:
    RSColorMapper(RS_COLOR_MAP_TYPE colormapType, float scale = 100.0, bool isHigh = true);

    // 非加权
    int m_applyColorMap(std::vector<float> &srcArray, const float rangeMin,const float rangeMax,
                        std::vector<st_ColorMapRGB_UCHAR> &dstColorArray, st_ColorMapRGB_UCHAR &bottomColor);

    // 加权
    int m_applyColorMap(std::vector<std::vector<float>> &srcArray, const std::vector<float> &aplhas,
                        const std::vector<float> &rangeMins, const std::vector<float> &rangeMaxs,
                        std::vector<st_ColorMapRGB_UCHAR> &dstColorArray, st_ColorMapRGB_UCHAR &bottomColor);

    void m_updateColorMapType(RS_COLOR_MAP_TYPE colormapType) {
        m_colormapType = colormapType;
    }

private:
    int m_linerInterp(int lowIndex, int highIndex, float index, float *pColorR,
                      float *pColorG, float *pColorB, float &r, float &g, float &b);

    int m_colorMap(const std::vector<float> &srcArray, std::vector<st_ColorMapRGB_UCHAR> &dstColorArray,
                   st_ColorMapRGB_UCHAR &bottomColor, int interpCnt, float *pColorR, float *pColorG, float *pColorB);

    int m_colorMap(const std::vector<std::vector<float>> &srcArray, const std::vector<float> &alphas,
                   std::vector<st_ColorMapRGB_UCHAR> &dstColorArray, st_ColorMapRGB_UCHAR &bottomColor, int interpCnt,
                   float *pColorR, float *pColorG, float *pColorB);


    RS_COLOR_MAP_TYPE m_colormapType;
    float m_scale;
    bool m_isHigh;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CUSTOM_ROBOSENSE_WEBSOCKET_WEBSOCKET_COLOR_MAPPER_H_
