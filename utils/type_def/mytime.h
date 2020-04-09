/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef CARTOGRAPHER_COMMON_TIME_H_
#define CARTOGRAPHER_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "port.h"

/*
预备知识:
c++11 提供了语言级别的时间函数.包括duration和time_point
duration是时间段,指的是某单位时间上的一个明确的tick(片刻数)：
3分钟->"3个1分钟",
1.5个"1/3秒" :1.5是tick,1/3秒是时间单位

time_point是一个duration和一个epoch(起点)的组合：
2017年5月4日是"自1970,01,01"以来的126200000秒数

common/time.h主要功能是提供时间转换函数：
*/

namespace Omega {
namespace common {

//719162 是0001年1月1日到1970年1月1日所经历的天数
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;     //微秒,us
using Time = UniversalTimeScaleClock::time_point;       //时间点

// Convenience functions to create common::Durations.
Duration FromSeconds(double seconds);               //将秒数seconds转为c++的duration实例对象
Duration FromMilliseconds(int64 milliseconds);      //毫秒

// Returns the given duration in seconds.
// 将的duration实例对象转为 秒数
double ToSeconds(Duration duration);

// Creates a time from a Universal Time Scale.
//将TUC时间(微秒)转化为c++的time_point对象
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
//将c++的time_point对象转为TUC时间,单位是us
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
//重载<<操作符,将time_point以string输出
std::ostream& operator<<(std::ostream& os, Time time);

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_
