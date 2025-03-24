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

#ifndef RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_COMMON_GEO_H_
#define RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_COMMON_GEO_H_

#include <algorithm>
#include "rs_perception/common/external/common.h"

namespace robosense {
namespace perception {

// 生成0~1的浮点数
inline float randUni() {
    return rand() / static_cast<double>(RAND_MAX);
}

inline float determinant(RsVector3f u, RsVector3f v) {
    float result = u.x * v.y - u.y * v.x;
    return result;
}

inline void clockwise(std::vector<RsVector3f> &polygons) {
    if (polygons.size() < 3) {
        return;
    }
    if (determinant(polygons[1] - polygons[0], polygons[2] - polygons[1]) > 0) {
        std::reverse(polygons.begin(), polygons.end());
    }
}

inline float distance(const RsVector3f &v1, const RsVector3f &v2) {
    float distance = sqrt(pow((v2.x - v1.x), 2) + pow((v2.y - v1.y), 2));
    return distance;
}

// checks if a point is within the triangle ABC :
// the point must be at the left of each edge -> be careful to the winding direction
inline bool collisionTrianglePoint(RsVector3f a, RsVector3f b, RsVector3f c, RsVector3f point) {
    RsVector3f ab = b - a;
    RsVector3f bc = c - b;
    RsVector3f ca = a - c;

    if (determinant(ab, point - a) > 0 &&
        determinant(bc, point - b) > 0 &&
        determinant(ca, point - c) > 0) {
        return true;
    } else {
        return false;
    }
}

inline std::vector<std::vector<RsVector3f> > triangulate(std::vector<RsVector3f> points) {
    // the ear clipping algorithm
    std::vector<std::vector<RsVector3f> > triangles;
    // a dynamic array that will store the points of the triangles :
    // if the triangle n is (An Bn Cn), then the points will be stored as
    // [A1,B1,C1,
    // A2,B2,C2,
    // A3,B3,C3...]
    std::vector<RsVector3f> initialPoints = points;
    if (points.size() < 3) {
        // let's make sure that the user don't feed the function with less than 3 points !
        return triangles;
    } else {
//        clockwise(points);
        bool triangleFound = true;

        while (points.size() != 0) {
            // run the algorithm until our polygon is empty
            if (!triangleFound) {
                // if we've looped once without finding any ear, the program is stuck,
                // the polygon is not triangulable for our algorithm
                // (likely to be a 8 shape or such self intersecting polygon)
                return triangles;
            }

            triangleFound = false;  // we want to find a new ear at each loop

            for (size_t i(0); i < points.size() - 2; i++) {
                // for each 3 consecutive points we check if it's an ear :
                // an ear is a triangle that wind in the right direction and
                // that do not contain any other point of the polygon
                if (!triangleFound) {
                    // if we still didn't find an ear
                    bool result = false;
                    if (determinant(points[i + 1] - points[i], points[i + 2] - points[i + 1]) < 0) {
                        // if the triangle winds in the right direction
                        result = true;
                        for (size_t j(0); j < initialPoints.size(); j++) {
                            // we check if there's no point inside it
                            if (collisionTrianglePoint(points[i + 2], points[i + 1],
                                                       points[i], initialPoints[j])) {
                                result = false;  // if I got a point in my triangle, then it's not an ear !
                            }
                        }
                    }

                    if (result) {
                        // now, we have found an ear :
                        triangleFound = true;

                        std::vector<RsVector3f> in_triangle(3);
                        in_triangle[0] = points[i];
                        in_triangle[1] = points[i + 1];
                        in_triangle[2] = points[i + 2];
                        triangles.push_back(in_triangle);

                        std::vector<RsVector3f> bufferArray;
                        for (size_t j(0); j < points.size(); j++) {
                            // then we delete the triangle in the points array :
                            // we already know that it's an ear, we don't need it anymore
                            if (j != i + 1) {
                                // we copiy all the points in a buffer array except the point we don't want
                                bufferArray.push_back(points[j]);
                            }
                        }
                        points = bufferArray;
                    }
                }
            }
        }
    }
    return triangles;  // we return the triangle array
}

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_RVIZ_DISPLAY_EXTERNAL_COMMON_GEO_H_
