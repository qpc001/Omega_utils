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
 * @brief Defines the EulerAnglesZXY class.
 */

#pragma once

// TODO(all): should use Angle class internally.

#include <cmath>

#include "Eigen/Geometry"

#include "math_utils.h"

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
 * @class EulerAnglesZXY
 *
 * Any orientation of a rigid body on a 3-D space can be achieved by
 * composing three rotations about the axes of an orthogonal coordinate system.
 * These rotations are said to be extrinsic if the axes are assumed to be
 * motionless, and intrinsic otherwise. Here, we use an intrinsic referential,
 * which is relative to the car's orientation.
 *
 * Our vehicle reference frame follows NovAtel's convention:
 * Right/Forward/Up (RFU) respectively for the axes x/y/z.
 * In particular, we describe the orientation of the car by three angles:
 * 1) the pitch, in (-pi/2, pi/2), corresponds to a rotation around the x-axis;
 * 2) the roll, in [-pi, pi), corresponds to a rotation around the y-axis;
 * 3) the yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
 * The pitch is zero when the car is level and positive when the nose is up.
 * The roll is zero when the car is level and positive when the left part is up.
 * The yaw is zero when the car is facing North, and positive when facing West.
 * In turn, in the world frame, the x/y/z axes point to East/North/Up (ENU).
 * These angles represent the rotation from the world to the vehicle frames.
 *
 * 对应`右-前-上` 的车体坐标系 x-y-z
 * 1) 俯仰角pitch：(-pi/2, pi/2) 绕x轴旋转， 车头翘起来则 pitch>0       (逆时针为正)
 * 2) 横滚角roll:[-pi, pi) 绕y轴旋转，从车尾向车头看，左侧翘起来，则roll>0 (逆时针为正)
 * 3) 偏航角yaw:[-pi, pi) 绕z轴转，指向西边为正(北偏西)                  (逆时针为正)
 *
 * 对应于东北天E-N-U导航坐标系
 * 这些角度，代表了从 `ENU导航坐标系` 转换到 `车体坐标系` 的变换
 *
 * 其中，从 `ENU导航坐标系` 转换到 `车体坐标系` 的对齐是指:
 *  1. 车体坐标系的 X轴 对齐 ENU坐标系的X轴， 即东侧
 *  2. 车体坐标系的 Y轴 对齐 ENU坐标系的Y轴， 即北侧
 *  3. 车体坐标系的 Z轴 对齐 ENU坐标系的Z轴， 即向上
 *
 *  此时，roll=0, pitch=0 , yaw=0
 *  但是，可以看到，车头，实际上是指向北面的，即坐标系对齐之后，车头是指向北面的=====>
 *  因此，在zxy（312）顺序-ENU 这种情况下， yaw角 和 车头相对于东方(也就是导航坐标系的x轴)的角度 是不相同的
 *  yaw=0 , 车头指向北方
 *  yaw=-pi/2 , 车头指向东方
 *
 * @brief Implements a class of Euler angles (actually, Tait-Bryan angles),
 * with intrinsic sequence ZXY.
 * 注意事项:
 *       1. 与欧拉角的顺序有关: 欧拉角转: 1. 四元数  2. 旋转矩阵
 *       2. 与欧拉角的顺序无关: 旋转矩阵(DCM)<=====>四元数(q)
 *
 * @param T Number type: double or float
 */
template<typename T>
class EulerAnglesZXY
{
public:
    /**
     * @brief Constructs an identity rotation.
     */
    EulerAnglesZXY()
        : roll_(0), pitch_(0), yaw_(0)
    {}

    /**
     * @brief Constructs a rotation using only yaw (i.e., around the z-axis).
     *
     * @param yaw The yaw of the car
     */
    explicit EulerAnglesZXY(T yaw)
        : roll_(0), pitch_(0), yaw_(yaw)
    {}

    /**
     * @brief Constructs a rotation using arbitrary roll, pitch, and yaw.
     *
     * @param roll The roll of the car
     * @param pitch The pitch of the car
     * @param yaw The yaw of the car
     */
    EulerAnglesZXY(T roll, T pitch, T yaw)
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
    EulerAnglesZXY(T qw, T qx, T qy, T qz)
        : roll_(std::atan2(static_cast<T>(2.0) * (qw * qy - qx * qz),
                           static_cast<T>(2.0) * (Square<T>(qw) + Square<T>(qz)) -
                               static_cast<T>(1.0))),
          pitch_(std::asin(static_cast<T>(2.0) * (qw * qx + qy * qz))),
          yaw_(std::atan2(static_cast<T>(2.0) * (qw * qz - qx * qy),
                          static_cast<T>(2.0) * (Square<T>(qw) + Square<T>(qy)) -
                              static_cast<T>(1.0)))
    {}

    /**
     * @brief Constructs a rotation from quaternion.
     * @param q Quaternion
     */
    explicit EulerAnglesZXY(const Eigen::Quaternion<T> &q)
        : EulerAnglesZXY(q.w(), q.x(), q.y(), q.z())
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

      T qw = cr * cp * cy - sr * sp * sy;
      T qx = cr * sp * cy - sr * cp * sy;
      T qy = cr * sp * sy + sr * cp * cy;
      T qz = cr * cp * sy + sr * sp * cy;
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

      // 构造从`ENU导航坐标系` 到 `body系`的旋转变换
      Eigen::Matrix<T,3,3> R_;
      R_(0,0)=cy*cr-sy*sp*sr;
      R_(0,1)=sy*cr+cy*sp*sr;
      R_(0,2)=-cp*sr;
      R_(1,0)=-sy*cp;
      R_(1,1)=cy*cp;
      R_(1,2)=sp;
      R_(2,0)=cy*sr+sy*sp*cr;
      R_(2,1)=sy*sr-cy*sp*cr;
      R_(2,2)=cp*cr;
      // 返回从`body系`到`ENU导航坐标系`的旋转变换
      return R_.transpose();
    }

private:
    T roll_;
    T pitch_;
    T yaw_;
};

using EulerAnglesZXYf = EulerAnglesZXY<float>;
using EulerAnglesZXYd = EulerAnglesZXY<double>;

template <typename T>
Eigen::Matrix<T,3,3> q2DCM(Eigen::Quaternion<T> q_){
  double qw=q_.w();
  double qx=q_.x();
  double qy=q_.y();
  double qz=q_.z();
  double qw2=qw*qw;
  double qx2=qx*qx;
  double qy2=qy*qy;
  double qz2=qz*qz;
  Eigen::Matrix<T,3,3> R_;
  R_(0,0)=qw2+qx2-qy2-qz2;
  R_(0,1)=2*(qx*qy-qw*qz);
  R_(0,2)=2*(qx*qz+qw*qy);
  R_(1,0)=2*(qx*qy+qw*qz);
  R_(1,1)=qw2-qx2+qy2-qz2;
  R_(1,2)=2*(qy*qz-qw*qx);
  R_(2,0)=2*(qx*qz-qw*qy);
  R_(2,1)=2*(qy*qz+qw*qx);
  R_(2,2)=qw2-qx2-qy2+qz2;
  return  R_;
}


}  // namespace math
}  // namespace common
}  // namespace Omega
