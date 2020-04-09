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
 * @brief Defines the Vec3d class.
 */

#pragma once

#include <cmath>
#include <string>
#include <cmath>
#include <Eigen/Core>
#include "absl/strings/str_cat.h"

/**
 * @namespace Omega::common::math
 * @brief Omega::common::math
 */
namespace Omega {
namespace common {
namespace math {

//constexpr double kMathEpsilon = 1e-10;

/**
 * @class Vec3d
 *
 * @brief Implements a class of 2-dimensional vectors.
 * @brief 向量，可作为3d点
 */

class Vec3d : public Eigen::Vector3d {
public:
    Vec3d(double x_,double y_,double z_) : Eigen::Vector3d(x_,y_,z_) {}

    void setX(double x_){
        this->x()=x_;
    }

    void setY(double y_){
        this->y()=y_;
    }

    void setZ(double z_){
        this->z()=z_;
    }

    double distanceTo(Vec3d &other){
        return std::sqrt(
            (this->x()-other.x())*(this->x()-other.x())+
                (this->y()-other.y())*(this->y()-other.y())+
                (this->z()-other.z())*(this->z()-other.z()));

    }

    std::string DebugString() const{
        return absl::StrCat("vec3d ( x = ", this->x(), " , y = ", this->y(), " , z= ",this->z(), " )");
    }

};

}  // namespace math
}  // namespace common
}  // namespace Omega