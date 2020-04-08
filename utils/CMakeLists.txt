cmake_minimum_required(VERSION 2.8)
project(utils)

#设置C++14
#判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++14")
endif(CMAKE_COMPILER_IS_GNUCXX)

#fundamental
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/fundamental
)
set(FUNDAMENTAL_LIB
        fundamental/vec2d.cc
        fundamental/search.cc)

add_library(common ${FUNDAMENTAL_LIB})
target_link_libraries(common glog absl::strings)

add_executable(${PROJECT_NAME}  main_test.cpp)
target_link_libraries(${PROJECT_NAME} glog absl::strings common)