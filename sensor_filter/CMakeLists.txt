cmake_minimum_required(VERSION 2.8.3)
project(sensor_filter)


 add_compile_options(-std=c++11)


find_package(Eigen3 REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
 add_library(${PROJECT_NAME} SHARED
   src/sensor_filter.cpp
 )

target_link_libraries(sensor_filter ${catkin_LIBRARIES})


