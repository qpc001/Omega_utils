/******************************************************************************
 * Copyright 2017 The Omega Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief Contains a number of helper functions related to quaternions.
 * The reader should refer to euler_angles_zxy.h for clarifications.
 * However, although the vehicle frame defined therein might change,
 * the definition of the heading of the car, used below, is permanently fixed:
 * 0 at East, pi/2 at North, -pi/2 at South.
 */

#pragma once

#include <cmath>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "euler_angles_zxy.h"
#include "math_utils.h"
//#include "modules/common/proto/geometry.pb.h"

/**
 * @namespace Omega::common::math
 * @brief Omega::common::math
 */
namespace Omega {
namespace common {
namespace math {

/*
 * @brief Returns heading (in radians) in [-PI, PI), with 0 being East.
 * Note that x/y/z is East/North/Up.
 * 输入: yaw对应的四元数
 * 输出: heading
 *
 * @param qw Quaternion w-coordinate
 * @param qx Quaternion x-coordinate
 * @param qy Quaternion y-coordinate
 * @param qz Quaternion z-coordinate
 *
 * @return Heading encoded by given quaternion
 */
inline double Quaternion_zxyToHeading(const double qw, const double qx,
                                  const double qy, const double qz) {
  EulerAnglesZXYd euler_angles(qw, qx, qy, qz);
  // euler_angles.yaw() is zero when the car is pointing North, but
  // the heading is zero when the car is pointing East.
  // * 需要注意的是:
  // *  1. 当车头指向东边的时候，heading应该是0，而此时的yaw=-pi/2
  // *  2. 当车头指向真北的时候，heading=pi/2，此时的yaw=0
  // 这里求的是heading = yaw + Pi/2
  return NormalizeAngle(euler_angles.yaw() + M_PI_2);
}

/*
 * @brief Returns heading (in radians) in [-PI, PI), with 0 being East.
 * Note that x/y/z is East/North/Up.
 *
 * @param q Eigen::Quaternion
 *
 * @return Heading encoded by given quaternion
 */
template <typename T>
inline T Quaternion_zxyToHeading(const Eigen::Quaternion<T> &q) {
  return static_cast<T>(Quaternion_zxyToHeading(q.w(), q.x(), q.y(), q.z()));
}

/*
 * @brief Returns a quaternion encoding a rotation with zero roll, zero pitch,
 * and the specified heading/yaw.
 * Note that heading is zero at East and yaw is zero at North.
 * Satisfies QuaternionToHeading(HeadingToQuaternion(h)) = h.
 *
 * 返回一个四元数，其中，roll=0，pitch=0，
 * 需要注意的是:
 *  1. 当车头指向东边的时候，heading应该是0，而此时的yaw=-pi/2
 *  2. 当车头指向真北的时候，heading=pi/2，此时的yaw=0
 *
 *  所以，使用heading来构造四元数的时候，需要减去pi/2, 来得到yaw
 *
 * @param heading The heading to encode in the rotation
 *
 * @return Quaternion encoding rotation by given heading
 */
template <typename T>
inline Eigen::Quaternion<T> HeadingToQuaternion_zxy(T heading) {
  // Note that heading is zero at East and yaw is zero at North.
  EulerAnglesZXY<T> euler_angles(heading - M_PI_2);
  return euler_angles.ToQuaternion();
}

/*
 * @brief Applies the rotation defined by a quaternion to a given vector.
 * Note that x/y/z is East/North/Up.
 *
 * @param orientation Quaternion
 * @param original Vector (in East-North-Up frame)
 *
 * @return Rotated vector
 */
//inline Eigen::Vector3d QuaternionRotate(const Quaternion &orientation,
//                                        const Eigen::Vector3d &original) {
//  Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
//                                       orientation.qy(), orientation.qz());
//  return static_cast<Eigen::Vector3d>(quaternion.toRotationMatrix() * original);
//}
//
//inline Eigen::Vector3d InverseQuaternionRotate(const Quaternion &orientation,
//                                               const Eigen::Vector3d &rotated) {
//  Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
//                                       orientation.qy(), orientation.qz());
//  return static_cast<Eigen::Vector3d>(quaternion.toRotationMatrix().inverse() *
//                                      rotated);
//}

}  // namespace math
}  // namespace common
}  // namespace Omega
