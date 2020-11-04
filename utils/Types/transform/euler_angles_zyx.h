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
 * @brief Defines the EulerAnglesZYX class.
 */

#pragma once

// TODO(all): should use Angle class internally.

#include <cmath>

#include "Eigen/Geometry"

#include "Operation/math_utils.h"

/**
 * @namespace Omega::common::math
 * @brief Omega::common::math
 */
namespace Omega
{
namespace common
{
namespace math
{

/**
 * @class EulerAnglesZYX
 *
 * 对应`前-右-下` 的车体坐标系 `x-y-z`
 * 描述: X轴指向前进方向，Y轴指向车体右侧，Z轴指向下方
 * 1) 俯仰角pitch:  (-pi/2, pi/2) 绕y轴旋转                     (逆时针为正)
 * 2) 横滚角roll:   [-pi, pi)     绕x轴旋转                     (逆时针为正)
 * 3) 偏航角yaw:    [-pi, pi)     绕z轴旋转                     (逆时针为正)
 *
 * 对应于`北-东-地` 即N-E-D导航坐标系
 * 这些角度，代表了从 `NED导航坐标系` 转换到 `车体坐标系` 的变换
 *
 * 其中，从 `NED导航坐标系` 转换到 `车体坐标系` 的对齐是指:
 *  1. 车体坐标系的 X轴 对齐 NED坐标系的X轴， 即北侧
 *  2. 车体坐标系的 Y轴 对齐 ENU坐标系的Y轴， 即东侧
 *  3. 车体坐标系的 Z轴 对齐 ENU坐标系的Z轴， 即向下
 *
 *  此时，roll=0, pitch=0 , yaw=0
 *  但是，可以看到，车头，实际上是指向北面的，即坐标系对齐之后，车头是指向北面的
 *
 * @brief Implements a class of Euler angles (actually, Tait-Bryan angles),
 * with intrinsic sequence ZYX.
 * 注意事项:
 *       1. 与欧拉角的顺序有关: 欧拉角转: 1. 四元数  2. 旋转矩阵
 *       2. 与欧拉角的顺序无关: 旋转矩阵(DCM)<=====>四元数(q)
 *
 * @param T Number type: double or float
 */
template<typename T>
class EulerAnglesZYX
{
public:
    /**
     * @brief Constructs an identity rotation.
     */
    EulerAnglesZYX()
        : roll_(0), pitch_(0), yaw_(0)
    {}

    /**
     * @brief Constructs a rotation using only yaw (i.e., around the z-axis).
     *
     * @param yaw The yaw of the car
     */
    explicit EulerAnglesZYX(T yaw)
        : roll_(0), pitch_(0), yaw_(yaw)
    {}

    /**
     * @brief Constructs a rotation using arbitrary roll, pitch, and yaw.
     *
     * @param roll The roll of the car
     * @param pitch The pitch of the car
     * @param yaw The yaw of the car
     */
    EulerAnglesZYX(T roll, T pitch, T yaw)
        : roll_(roll), pitch_(pitch), yaw_(yaw)
    {}

    /**
     * @brief Constructs a rotation using components of a quaternion.
     *
     * @param qw Quaternion w-coordinate
     * @param qx Quaternion x-coordinate
     * @param qy Quaternion y-coordinate
     * @param qz Quaternion z-coordinate
     */
    EulerAnglesZYX(T qw, T qx, T qy, T qz)
    {
        // roll (x-axis rotation)
        T sinr_cosp = 2 * (qw * qx + qy * qz);
        T cosr_cosp = 1 - 2 * (qx * qx + qy * qy);

        T roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        T pitch=0;
        T sinp = 2 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        T siny_cosp = 2 * (qw * qz + qx * qy);
        T cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        T  yaw = std::atan2(siny_cosp, cosy_cosp);

        roll_=roll;
        pitch_=pitch;
        yaw_=yaw;
    }

    /**
     * @brief Constructs a rotation from quaternion.
     * @param q Quaternion
     */
    explicit EulerAnglesZYX(const Eigen::Quaternion<T> &q)
        : EulerAnglesZYX(q.w(), q.x(), q.y(), q.z())
    {}

    /**
     * @brief Getter for roll_
     * @return The roll of the car
     */
    T roll() const
    { return roll_; }

    /**
     * @brief Getter for pitch_
     * @return The pitch of the car
     */
    T pitch() const
    { return pitch_; }

    /**
     * @brief Getter for yaw_
     * @return The yaw of the car
     */
    T yaw() const
    { return yaw_; }

    /**
     * @brief Normalizes roll_, pitch_, and yaw_ to [-PI, PI).
     */
    void Normalize()
    {
      roll_ = NormalizeAngle(roll_);
      pitch_ = NormalizeAngle(pitch_);
      yaw_ = NormalizeAngle(yaw_);
    }

    /**
     * @brief Verifies the validity of the specified rotation.
     * 检查pitch是否在范围之内，否则产生 gimbol lock
     * @return True iff -PI/2 < pitch < PI/2
     */
    bool IsValid()
    {
      Normalize();
      return pitch_ < M_PI_2 && pitch_ > -M_PI_2;
    }

    /**
     * @brief Converts to a quaternion with a non-negative scalar part
     * 欧拉角转四元数 : 这里返回的是 从 `body系` 到 `ENU导航坐标系` 的变换 , 因为从 `body系` 到 `ENU导航坐标系` 更加通用
     * (尽管欧拉角是从 `ENU导航坐标系` 到 `body系` 的)
     * 代码实现: From wiki [四元数与欧拉角之间的转换](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion)
     * @return Quaternion encoding this rotation.
     */
    Eigen::Quaternion<T> ToQuaternion() const
    {
      T coeff = static_cast<T>(0.5);
      T r = roll_ * coeff;
      T p = pitch_ * coeff;
      T y = yaw_ * coeff;

      T sr = std::sin(r);
      T sp = std::sin(p);
      T sy = std::sin(y);

      T cr = std::cos(r);
      T cp = std::cos(p);
      T cy = std::cos(y);

      T qw = cr * cp * cy + sr * sp * sy;
      T qx = sr * cp * cy - cr * sp * sy;
      T qy = cr * sp * cy + sr * cp * sy;
      T qz = cr * cp * sy - sr * sp * cy;
      if (qw < 0.0) {
        return {-qw, -qx, -qy, -qz};
      }
      return {qw, qx, qy, qz};
    }

    //TODO : 欧拉角转旋转矩阵
    Eigen::Matrix<T, 3, 3> ToRotationMatrix() const
    {
      double cr=cos(roll_);
      double cp=cos(pitch_);
      double cy=cos(yaw_);
      double sr=sin(roll_);
      double sp=sin(pitch_);
      double sy=sin(yaw_);

      // 构造从`NED导航坐标系` 到 `body系`的旋转变换
      Eigen::Matrix<T,3,3> R_;
      R_(0,0)=cp*cy;
      R_(0,1)=cp*sy;
      R_(0,2)=-sp;
      R_(1,0)=-cr*sy+sr*sp*cy;
      R_(1,1)=cr*cy+sr*sp*sy;
      R_(1,2)=sr*cp;
      R_(2,0)=sr*sy+cr*sp*cy;
      R_(2,1)=-sr*cy+cr*sp*sy;
      R_(2,2)=cr*cp;
      // 返回从`body系`到`NED导航坐标系`的旋转变换
      return R_.transpose();
    }

private:
    T roll_;
    T pitch_;
    T yaw_;
};

using EulerAnglesZYXf = EulerAnglesZYX<float>;
using EulerAnglesZYXd = EulerAnglesZYX<double>;

}  // namespace math
}  // namespace common
}  // namespace Omega
