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

#include "vec3d.h"

#include <cmath>

#include "absl/strings/str_cat.h"
#include "../log/log.h"

namespace Omega
{
namespace common
{
Vec3d Vec3d::operator+(const Vec3d &other) const {
    return Vec3d(this->x() + other.x(), this->y() + other.y(),this->z()+other.z());
}

Vec3d Vec3d::operator-(const Vec3d &other) const {
    return Vec3d(this->x() - other.x(), this->y() - other.y(),this->z()-other.z());
}

Vec3d Vec3d::operator*(const double ratio) const {
    return Vec3d(this->x() * ratio, this->y() * ratio, this->z()*ratio);
}

Vec3d Vec3d::operator/(const double ratio) const {
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    return Vec3d(this->x() / ratio, this->y() / ratio, this->z()/ratio);
}

Vec3d &Vec3d::operator+=(const Vec3d &other) {
    this->x() += other.x();
    this->y() += other.y();
    this->z() += other.z();
    return *this;
}

Vec3d &Vec3d::operator-=(const Vec3d &other) {
    this->x() -= other.x();
    this->y() -= other.y();
    this->z() -= other.z();
    return *this;
}

Vec3d &Vec3d::operator*=(const double ratio) {
    this->x() *= ratio;
    this->y() *= ratio;
    this->z() *= ratio;
    return *this;
}

Vec3d &Vec3d::operator/=(const double ratio) {
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    this->x() /= ratio;
    this->y() /= ratio;
    this->z() /= ratio;
    return *this;
}

bool Vec3d::operator==(const Vec3d &other) const {
    return (std::abs(this->x() - other.x()) < kMathEpsilon &&
        std::abs(this->y() - other.y()) < kMathEpsilon &&
        std::abs(this->z() - other.z())< kMathEpsilon);
}
}  // namespace common
}  // namespace Omega
