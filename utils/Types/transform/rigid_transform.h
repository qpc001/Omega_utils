/*
 * Copyright 2016 The Omega Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef Omega_TRANSFORM_RIGID_TRANSFORM_H_
#define Omega_TRANSFORM_RIGID_TRANSFORM_H_

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "Operation/math_utils.h"
#include "Types/vec/vec2d.h"
#include "Types/vec/vec3d.h"

/** 刚体变换2D/3D 类
 *
 */

namespace Omega
{
namespace common
{
namespace transform
{
//2D
template<typename FloatType>
class Transform2d : public Eigen::Isometry2d
{
public:
    using Vector = Eigen::Matrix<FloatType, 2, 1>;
    using Rotation2D = Eigen::Rotation2D<FloatType>;

    Transform2d() : Eigen::Isometry2d(Eigen::Isometry2d::Identity()) {}
    Transform2d( Eigen::Isometry2d T_) : Eigen::Isometry2d(T_) {}

//    Transform2d(const Vector &translation, const Rotation2D &rotation)
//        : translation_(translation), rotation_(rotation)
//    {}
//    Transform2d(const Vector &translation, const double rotation)
//        : translation_(translation), rotation_(rotation)
//    {}

    Transform2d<FloatType> inv() {
        return Transform2d(this->inverse());
    }

    static Transform2d<FloatType> Identity()
    { return Transform2d<FloatType>(); }

    // 设置平移
    void setTranslate(Vector t){
        this->translate(t);
    }

    // 求旋转角度， 逆时针为正
    double getAngle() const{
        return atan2(this->rotation().matrix()(0,1),this->rotation().matrix()(0,0));
    }

    // 旋转
    void setRotate(double angle_rad) {
        double s=sin(angle_rad);
        double c=cos(angle_rad);
        Eigen::Matrix2d r;
        r << c , s , -s , c;
        this->rotate(r);
    }

//    //旋转角度归一化到[-pi;pi]
//    double normalized_angle() const
//    {
//        return Omega::common::math::NormalizeAngleDifference(rotation().angle());
//    }

    std::string DebugString() const
    {
        std::string out;
        out.append("{ t: [");
        out.append(std::to_string(this->translation().x()));
        out.append(", ");
        out.append(std::to_string(this->translation().y()));
        out.append("], r: [");
        out.append(std::to_string(Omega::common::math::NormalizeAngle(getAngle())));
        out.append("] }");
        return out;
    }

private:
//    Vector translation_;
//    Rotation2D rotation_;
};

// 操作符,两个2D变换之间乘法
template<typename FloatType>
Transform2d<FloatType> operator*(const Transform2d<FloatType> &lhs,
                                 const Transform2d<FloatType> &rhs)
{
    return Transform2d<FloatType>(
        lhs.rotation() * rhs.translation() + lhs.translation(),
        lhs.rotation() * rhs.rotation());
}

// 操作符，2D变换和点的变换
//template<typename FloatType>
//typename Transform2d<FloatType>::Vector operator*(
//    const Transform2d<FloatType> &rigid,
//    const typename Transform2d<FloatType>::Vector &point)
//{
//    return rigid.rotation() * point + rigid.translation();
//}

template<typename  T, typename FloatType>
T operator*(const Transform2d<FloatType> &rigid,const T pt){
    return T(rigid.rotation()*pt+rigid.translation());
}

// This is needed for gmock.
template<typename T>
std::ostream &operator<<(std::ostream &os,
                         const Omega::common::transform::Transform2d<T> &rigid)
{
    os << rigid.DebugString();
    return os;
}

using Transform2dd = Transform2d<double>;
using Transform2df = Transform2d<float>;

//=====================================================================
///3D
template<typename FloatType>
class Transform3d : public Eigen::Isometry3d
{
public:
    using Vector = Eigen::Matrix<FloatType, 3, 1>;
    using Quaternion = Eigen::Quaternion<FloatType>;
    using AngleAxis = Eigen::AngleAxis<FloatType>;

    //构造函数 1.平移+四元数 2.平移+轴角
    Transform3d() : Eigen::Isometry3d(Eigen::Isometry3d::Identity()) {}
    Transform3d(Eigen::Isometry3d &&T_) : Eigen::Isometry3d(T_) {}
    //Transform3d(Eigen::Isometry3d T_) : Eigen::Isometry3d(T_) {}
    Transform3d(const Vector &translation, const Quaternion &rotation)
    : Eigen::Isometry3d(rotation,translation)
    {}
    Transform3d(const Vector &translation, const AngleAxis &rotation)
    : Eigen::Isometry3d(rotation,translation)
    {}


    static Transform3d<FloatType> Identity()
    { return Transform3d<FloatType>(); }

    template<typename OtherType>
    Transform3d<OtherType> cast() const
    {
        return Transform3d<OtherType>(this->template cast<OtherType>());
    }

//    //从std::array中获取值
//    static Transform3d FromArrays(const std::array<FloatType, 4> &rotation,
//                                  const std::array<FloatType, 3> &translation)
//    {
//        return Transform3d(Eigen::Map<const Vector>(translation.data()),
//                           Eigen::Quaternion<FloatType>(rotation[0], rotation[1],
//                                                        rotation[2], rotation[3]));
//    }

    // 获取平移
    Vector getTranslation(){
        return this->translation();
    }

    // 设置平移
    void setTranslation(Vector t){
//        this->translate(t);
        this->translation()=t;
    }

    // 获取旋转 q
    Eigen::Quaternion<FloatType> getRotation() const {
        return Eigen::Quaternion<FloatType>(this->rotation());
    }

    Eigen::Matrix<FloatType,3,3> getRotationMatrix() const{
        return this->rotation().matrix();
    }

    // 设置旋转
    void setRotation(Eigen::Quaternion<FloatType> q_) {
        this->rotate(q_);
    }

    void setRotation(Eigen::Matrix<FloatType,3,3> R_) {
        this->rotate(R_);
    }

    //求逆变换
    Transform3d inv() const
    {
        //const Quaternion rotation = rotation_.conjugate();
        //const Vector translation = -(rotation * translation_);
        return Transform3d(this->inverse());
    }

    std::string DebugString() const
    {
        std::string out;
        out.append("{ t: [");
        out.append(std::to_string(translation().x()));
        out.append(", ");
        out.append(std::to_string(translation().y()));
        out.append(", ");
        out.append(std::to_string(translation().z()));
        out.append("], q: [");
        out.append(std::to_string(getRotation().w()));
        out.append(", ");
        out.append(std::to_string(getRotation().x()));
        out.append(", ");
        out.append(std::to_string(getRotation().y()));
        out.append(", ");
        out.append(std::to_string(getRotation().z()));
        out.append("] }");
        return out;
    }

//    bool IsValid() const
//    {
//        return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
//            !std::isnan(translation_.z()) &&
//            std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
//    }

private:
};
//操作符， 两个3D变换之间乘法
template<typename FloatType>
Transform3d<FloatType> operator*(const Transform3d<FloatType> &lhs,
                                 const Transform3d<FloatType> &rhs)
{
    return Transform3d<FloatType>(
        lhs.rotation() * rhs.translation() + lhs.translation(),
        (lhs.rotation() * rhs.rotation()).normalized());
}

// T: 用于传参为Omega::common::Vec3d 的时候的
template <typename T,typename FloatType>
T operator*(const Transform3d<FloatType> &rigid,T pt){
    return T(rigid.rotation()*pt+rigid.translation());
}

// This is needed for gmock.
template<typename T>
std::ostream &operator<<(std::ostream &os,
                         const Omega::common::transform::Transform3d<T> &rigid)
{
    os << rigid.DebugString();
    return os;
}

using Transform3dd = Transform3d<double>;
using Transform3df = Transform3d<float>;

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
// 欧拉角转四元数
//Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

}  // namespace common
}  // namespace transform
}  // namespace Omega

#endif  // Omega_TRANSFORM_RIGID_TRANSFORM_H_
