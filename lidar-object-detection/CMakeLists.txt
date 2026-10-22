cmake_minimum_required(VERSION 3.10)
project(lidar_stop CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ydlidar_sdk REQUIRED)   # installed by: YDLidar-SDK -> cmake .. && make && sudo make install
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(lidar_stop lidar_stop.cpp)
target_include_directories(lidar_stop PRIVATE ${YDLIDAR_SDK_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_directories(lidar_stop PRIVATE ${YDLIDAR_SDK_LIBRARY_DIRS})
target_link_libraries(lidar_stop PRIVATE ${YDLIDAR_SDK_LIBRARIES} ${OpenCV_LIBS} Threads::Threads)
