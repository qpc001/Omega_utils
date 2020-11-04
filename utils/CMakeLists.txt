cmake_minimum_required(VERSION 2.8)
project(utils)

#设置C++14
#判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++14")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(TOP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../)
file(GLOB_RECURSE HEADER_FILES  "*.h" "*.hpp")

# 在最下面的"target_include_directories"也有这个include_的功能
# include这些头文件路径
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/Operation
        ${CMAKE_CURRENT_SOURCE_DIR}/Types
        ${CMAKE_CURRENT_SOURCE_DIR}/Log
)

# 需要编译的头文件
set(FUNDAMENTAL_LIB
        Types/vec/vec2d.cc
        Types/vec/vec3d.cc
        Types/transform/rigid_transform.cc
        Operation/search.cc
        Operation/matrix_operations.cc
        Operation/math_utils.cc
        Operation/integral.cc
        StringHexByte/StringHexByte.cpp)

# 输出文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Omega_utils/Lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Omega_utils/Lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Omega_utils/Bin)

# 编译成库
add_library(common ${FUNDAMENTAL_LIB} ${HEADER_FILES})
add_library(Omega::common ALIAS common)
target_link_libraries(common glog absl::strings)
target_include_directories(common PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR})  #解决引用库时的头文件依赖问题

# 使用说明
add_executable(${PROJECT_NAME}  main_test.cpp)
target_link_libraries(${PROJECT_NAME} glog absl::strings common)
