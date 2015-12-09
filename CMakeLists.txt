cmake_minimum_required(VERSION 2.8.3)
project(surfel_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_CXX_FLAGS "-fopenmp -O3 -msse2 -msse3 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

catkin_package(
   INCLUDE_DIRS include
)

file (GLOB HDRS include/*.h)

add_executable(surfel_publisher
               ${HDRS}
               src/surfel_publisher.cpp
)

add_executable(surfel_exporter
               ${HDRS}
               src/surfel_exporter.cpp
)

target_link_libraries(surfel_publisher
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
)

target_link_libraries(surfel_exporter
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
)
