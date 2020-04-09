cmake_minimum_required(VERSION 3.0)

if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif(POLICY CMP0048)

project(proj)

#设置C++14
#判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++14")
endif(CMAKE_COMPILER_IS_GNUCXX)

#osqp
find_package(osqp REQUIRED)

#gtest
add_subdirectory(gtest)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

#absl
add_subdirectory(abseil-cpp)
include_directories(abseil-cpp/absl)

#glog
add_subdirectory(glog)
include_directories(glog/src/)

##uitls
add_subdirectory(utils)
include_directories(utils)

#test
add_subdirectory(test)

#target_link_libraries(${PROJECT_NAME} )