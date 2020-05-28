cmake_minimum_required(VERSION 2.8)
project(utils_test)

#设置C++14
#判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++14")
endif(CMAKE_COMPILER_IS_GNUCXX)

#osqp
#find_package(osqp REQUIRED)
#include_directories(${osqp_INCLUDE_DIRS})

# include这些头文件路径
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils/fundamental
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils/type_def
)
#include_directories("/usr/include/eigen3")

#fundamental/vec2d_test.cc
add_executable(${PROJECT_NAME}
        vec2d_test.cc
        search_test.cc
        matrix_operations_test.cc
        math_utils_test.cc
        integral_test.cc
        euler_angles_zxy_test.cc
        quaternion_zxy_test.cc)

#target_link
target_link_libraries(${PROJECT_NAME} gtest gtest_main  common)
target_link_libraries(${PROJECT_NAME} PRIVATE )
#target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqp)
