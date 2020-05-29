# How to use

## download

```
git clone https://github.com/qpc001/Omega_utils.git
```

## Move to your Ros workspace or your Project

```
mv Omega_utils/ YOUR-PATH/omega_utils
```

## Build

### Plain C++

```
mkdir build
cd build
cmake -DENABLE_ROS=OFF ..
make -j4
```

### ROS Package

首先需要将其移动到ros的catkin_ws/src中，作为一个Package

需要修改CMakeLists.txt中的`option(ENABLE_ROS "option for ROS" ON)`为ON

然后作为Package编译即可

## ROS其他Pacakge引用此库

假设有新的一个Package `aaa_test`

![](README/2020-05-29-18-49-40.png)

各个文件内容如下:

### aaa_test.cpp

```C++
#include <ros/ros.h>
#include "type_def/vec2d.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aaa_test");
    ros::NodeHandle nh;

    Omega::common::Vec2d a(2,3);
    std::cout<<a.DebugString()<<std::endl;

    ROS_INFO("Hello world!");
}
```

### CMakeLists.txt(in aaa_test)

```c
cmake_minimum_required(VERSION 2.8.3)
project(aaa_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #找到Package名
  omega_utils
  roscpp
)

catkin_package(
  CATKIN_DEPENDS omega_utils roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aaa_test aaa_test.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    common
    )
```

### package.xml(in aaa_test)

```xml
<?xml version="1.0"?>
<package format="2">
  <name>aaa_test</name>
  <version>0.0.0</version>
  <description>The aaa_test package</description>

  <maintainer email="xxx@todo.todo">autoware</maintainer>

  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>

  <!--  三个都要 -->
  <build_depend>omega_utils</build_depend>
  <build_export_depend>omega_utils</build_export_depend>
  <exec_depend>omega_utils</exec_depend>

  <export>
    <!-- Other tools can request additional information be placed here -->
  </export>
</package>
```

### Test

```
catkin_make
source ./devel/setup.bash 
rosrun aaa_test aaa_test 
```

OUTPUT:

```
vec2d ( x = 2 , y = 3 )
```