cmake_minimum_required(VERSION 2.8)
project(utils)

#设置C++14
#判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++14")
endif(CMAKE_COMPILER_IS_GNUCXX)

# include这些头文件路径
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/fundamental
        ${CMAKE_CURRENT_SOURCE_DIR}/type_def
)

# 需要编译的头文件
set(FUNDAMENTAL_LIB
        type_def/vec2d.cc
        type_def/vec3d.cc
        type_def/transform/rigid_transform.cc
        fundamental/search.cc
        fundamental/matrix_operations.cc
        fundamental/math_utils.cc
        fundamental/integral.cc
        type_def/transform/euler_angles_zxy.h)

# 编译成库
add_library(common ${FUNDAMENTAL_LIB})
target_link_libraries(common glog absl::strings)

# 使用说明
add_executable(${PROJECT_NAME}  main_test.cpp)
target_link_libraries(${PROJECT_NAME} glog absl::strings common)