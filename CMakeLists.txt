cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(LidarLineDetection)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)


# 添加头文件搜索路径
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)

# 添加可执行文件
add_executable(TestLidarLineDetection src/lidar_test_main.cpp)

# 添加共享库
add_library(LidarLineDetection SHARED src/lidar_line_detection.cpp)

# 链接库
target_link_libraries(TestLidarLineDetection LidarLineDetection ${OpenCV_LIBS})
target_link_libraries(LidarLineDetection ${OpenCV_LIBS})