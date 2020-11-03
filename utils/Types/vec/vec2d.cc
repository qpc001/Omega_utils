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

#include "vec2d.h"

#include <cmath>

#include "absl/strings/str_cat.h"
#include "Log/log.h"

namespace Omega
{
namespace common
{
Vec2d Vec2d::operator+(const Vec2d &other) const {
    return Vec2d(this->x() + other.x(), this->y() + other.y());
}

Vec2d Vec2d::operator-(const Vec2d &other) const {
    return Vec2d(this->x() - other.x(), this->y() - other.y());
}

Vec2d Vec2d::operator*(const double ratio) const {
    return Vec2d(this->x() * ratio, this->y() * ratio);
}

Vec2d Vec2d::operator/(const double ratio) const {
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    return Vec2d(this->x() / ratio, this->y() / ratio);
}

Vec2d &Vec2d::operator+=(const Vec2d &other) {
    this->x() += other.x();
    this->y() += other.y();
    return *this;
}

Vec2d &Vec2d::operator-=(const Vec2d &other) {
    this->x() -= other.x();
    this->y() -= other.y();
    return *this;
}

Vec2d &Vec2d::operator*=(const double ratio) {
    this->x() *= ratio;
    this->y() *= ratio;
    return *this;
}

Vec2d &Vec2d::operator/=(const double ratio) {
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    this->x() /= ratio;
    this->y() /= ratio;
    return *this;
}

bool Vec2d::operator==(const Vec2d &other) const {
    return (std::abs(this->x() - other.x()) < kMathEpsilon &&
        std::abs(this->y() - other.y()) < kMathEpsilon);
}

}  // namespace common
}  // namespace Omega
