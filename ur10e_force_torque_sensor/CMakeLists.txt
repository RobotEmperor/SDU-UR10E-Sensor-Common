CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )
 
PROJECT ( "ur10e_force_torque_sensor" )
SET ( PROJECT_VERSION_MAJOR 0 )
SET ( PROJECT_VERSION_MINOR 1 )

SET ( CMAKE_C_COMPILER "gcc" )
 
INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/include)

add_compile_options(-std=c++11)
 
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)


set(LIB_HEADER_FILES
        ${CMAKE_SOURCE_DIR}/include/ur10e_force_torque_sensor/ur10e_force_torque_sensor.h
)

INCLUDE_DIRECTORIES (${yaml_cpp_INCLUDE_DIRS})

set( ur10e_force_torque_sensor_SOURCE_FILES 
  src/ur10e_force_torque_sensor.cpp
)
link_directories(/usr/local/lib)
add_library(ur10e_force_torque_sensor SHARED ${ur10e_force_torque_sensor_SOURCE_FILES})
target_link_libraries(ur10e_force_torque_sensor sdu_math sensor_filter ${yaml_cpp_LIBRARIES})


if (CMAKE_COMPILER_IS_GNUCXX)
		# Give error in Linux if library is not fully linked (like Windows)
		set_target_properties(ur10e_force_torque_sensor PROPERTIES LINK_FLAGS -Wl,--no-undefined)
 
	endif()

	if(NOT DEFINED LIB_INSTALL_DIR)
		set(LIB_INSTALL_DIR "lib")
	endif()
	if(NOT DEFINED BIN_INSTALL_DIR)
		set(BIN_INSTALL_DIR "bin")
	endif()
	if(NOT DEFINED INCLUDE_INSTALL_DIR)
		set(INCLUDE_INSTALL_DIR "include")
	else()
		set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_DIR}")
	endif()

	install(TARGETS ur10e_force_torque_sensor
			LIBRARY DESTINATION ${LIB_INSTALL_DIR}
			ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
			RUNTIME DESTINATION ${BIN_INSTALL_DIR}
			INCLUDES DESTINATION ${INCLUDE_INSTALL_DIR}
			)

        install(
			FILES
			${LIB_HEADER_FILES}
			DESTINATION ${INCLUDE_INSTALL_DIR}/ur10e_force_torque_sensor
			COMPONENT ur10e_force_torque_sensor
	)

 
 
