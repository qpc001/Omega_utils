cmake_minimum_required(VERSION 3.0)
project(omega_utils)
#if(POLICY CMP0048)
#    cmake_policy(SET CMP0048 NEW)
#endif(POLICY CMP0048)

#设置C++14
#判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++14")
endif(CMAKE_COMPILER_IS_GNUCXX)

#####################ROS###############################

#ROS下作为Package的时候，需要设置为ON，然后删除 build/ deve/ 文件夹
#然后重新编译
option(ENABLE_ROS "option for ROS" OFF)
add_definitions(-DENABLE_ROS)

if(ENABLE_ROS)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

    catkin_package(
        # 这三个必须要有
        INCLUDE_DIRS utils          #库的头文件, 这样才能在B_package中 include 这个库的头文件
        LIBRARIES common            #库名
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
        )
endif()

####################Plain CMake#########################

#list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
    utils
    ${catkin_INCLUDE_DIRS}
    )

if(NOT ENABLE_ROS)
    #gtest
    add_subdirectory(third_party/gtest)
    enable_testing()
    include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

    #glog
    add_subdirectory(third_party/glog)
    include_directories(${glog_SOURCE_DIR}/src/)

    #gflag
    add_subdirectory(third_party/gflags)
    include_directories(${gflags_SOURCE_DIR}/src/)
endif()

#absl
add_subdirectory(third_party/abseil-cpp)
include_directories(${abseil-cpp_SOURCE_DIR}/absl)

#Eigen
include(cmake/FindEigen3.cmake)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

##uitls
add_subdirectory(utils)
include_directories(utils)
