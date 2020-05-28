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

#list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#osqp
#find_package(osqp REQUIRED)
#include_directories(${osqp_INCLUDE_DIRS})

#gtest
add_subdirectory(third_party/gtest)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

#absl
add_subdirectory(third_party/abseil-cpp)
include_directories(${abseil-cpp_SOURCE_DIR}/absl)

#glog
add_subdirectory(third_party/glog)
include_directories(${glog_SOURCE_DIR}/src/)

#gflag
add_subdirectory(third_party/gflags)
include_directories(${gflags_SOURCE_DIR}/src/)

#Eigen
include(cmake/FindEigen3.cmake)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

##uitls
add_subdirectory(utils)
include_directories(utils)

#test
add_subdirectory(test)


#target_link_libraries(${PROJECT_NAME} )
