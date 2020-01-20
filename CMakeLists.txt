CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )
 
PROJECT ( "sdu_ur10e_sensor_common" )
SET ( PROJECT_VERSION_MAJOR 0 )
SET ( PROJECT_VERSION_MINOR 1 )

SET ( CMAKE_C_COMPILER "gcc" )
 
INCLUDE_DIRECTORIES (${PROJECT_NAME}/include)

add_compile_options(-std=c++11)
 
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})

set(LIB_HEADER_FILES
        sensor_filter/include/sensor_filter/sensor_filter.h
        ur10e_force_torque_sensor/include/ur10e_force_torque_sensor/ur10e_force_torque_sensor.h
)

INCLUDE_DIRECTORIES (sensor_filter/include)
INCLUDE_DIRECTORIES (ur10e_force_torque_sensor/include)


set( sensor_filter_SOURCE_FILES 
  sensor_filter/src/sensor_filter.cpp
)

add_library(sensor_filter SHARED ${sensor_filter_SOURCE_FILES} ${LIB_HEADER_FILES})

set( ur10e_force_torque_sensor_SOURCE_FILES 
  ur10e_force_torque_sensor/src/ur10e_force_torque_sensor.cpp
)

add_library(ur10e_force_torque_sensor SHARED ${ur10e_force_torque_sensor_SOURCE_FILES}  ${LIB_HEADER_FILES})
target_link_libraries(ur10e_force_torque_sensor sdu_math sensor_filter)


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
install(TARGETS sensor_filter
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
install(
			FILES
			${LIB_HEADER_FILES}
			DESTINATION ${INCLUDE_INSTALL_DIR}/sensor_filter
			COMPONENT sensor_filter
	)


 
 
