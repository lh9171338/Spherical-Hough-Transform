########### 版本要求
cmake_minimum_required(VERSION 3.5)

########### 项目名称
project(SphericalHoughTransform)

###########  c++版本
set(CMAKE_CXX_STANDARD 14)

###########  编译模式
set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to release")

###########  添加库
# opncv库
find_package(OpenCV REQUIRED)

########### 添加头文件
include_directories(include)

########### 编译库
add_library(pointcloud src/pointcloud.cpp)
add_library(hough src/hough.cpp)
add_library(SphericalCamera src/SphericalCamera.cpp)
target_link_libraries(pointcloud ${OpenCV_LIBS})
target_link_libraries(hough pointcloud ${OpenCV_LIBS})
target_link_libraries(SphericalCamera pointcloud ${OpenCV_LIBS})

###########  设置可执行文件
add_executable(${PROJECT_NAME} main.cpp)

###########  设置链接库
target_link_libraries(${PROJECT_NAME} pointcloud hough SphericalCamera ${Dependencies})
