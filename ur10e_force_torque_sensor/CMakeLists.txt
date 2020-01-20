cmake_minimum_required(VERSION 2.8.3)
project(ur10e_force_torque_sensor)


 add_compile_options(-std=c++11)
 

find_package(Eigen3 REQUIRED)
find_package(sensor_filter)



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
   src/ur10e_force_torque_sensor.cpp
 )

target_link_libraries(ur10e_force_torque_sensor sdu_math sensor_filter)
