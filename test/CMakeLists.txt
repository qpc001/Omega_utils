cmake_minimum_required(VERSION 2.8)
project(utils_test)

#设置C++14
#判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++14")
endif(CMAKE_COMPILER_IS_GNUCXX)

#库目录
#set(TOP_DIR ${CMAKE_CURRENT_LIST_DIR}/../)

#fundamental
#include_directories(
#        ${TOP_DIR}/utils/fundamental
#)

#gtest
#set(GTEST_ROOT ${PROJECT_SOURCE_DIR}/../gtest/googletest)
#include_directories(${GTEST_ROOT}/include)
#message(${GTEST_ROOT}/include)
#find_package(gtest)
#include_directories(../gtest/googletest/include)

#fundamental/vec2d_test.cc
add_executable(${PROJECT_NAME}  vec2d_test.cc)
#target_link
target_link_libraries(${PROJECT_NAME} gtest gtest_main common)
#add_test(NAME {PROJECT_NAME} COMMAND ${PROJECT_NAME})
