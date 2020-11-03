cmake_minimum_required(VERSION 2.8)
project(utils)

#设置C++14
#判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++14")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(TOP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../)

# include这些头文件路径
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/Operation
        ${CMAKE_CURRENT_SOURCE_DIR}/Types
        ${CMAKE_CURRENT_SOURCE_DIR}/Log
)

#Eigen
#include(${TOP_DIR}/cmake/eigen.cmake)
#include_directories("/usr/include/eigen3")

# 需要编译的头文件
set(FUNDAMENTAL_LIB
        Types/vec/vec2d.cc
        Types/vec/vec3d.cc
        Types/transform/rigid_transform.cc
        Operation/search.cc
        Operation/matrix_operations.cc
        Operation/math_utils.cc
        Operation/integral.cc
#        type_def/transform/euler_angles_zxy.h
        StringHexByte/StringHexByte.cpp)

# 编译成库
add_library(common ${FUNDAMENTAL_LIB})
target_link_libraries(common glog absl::strings)

# 使用说明
add_executable(${PROJECT_NAME}  main_test.cpp)
target_link_libraries(${PROJECT_NAME} glog absl::strings common)
