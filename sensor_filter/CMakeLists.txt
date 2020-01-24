CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )
 
PROJECT ( "sensor_filter" )
SET ( PROJECT_VERSION_MAJOR 0 )
SET ( PROJECT_VERSION_MINOR 1 )

SET ( CMAKE_C_COMPILER "gcc" )
 
INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/include)

add_compile_options(-std=c++11)
 
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})


set( sensor_filter_SOURCE_FILES 
  src/sensor_filter.cpp
)
add_library(sensor_filter SHARED ${sensor_filter_SOURCE_FILES} ${LIB_HEADER_FILES})

if (CMAKE_COMPILER_IS_GNUCXX)
		# Give error in Linux if library is not fully linked (like Windows)
                set_target_properties(sensor_filter PROPERTIES LINK_FLAGS -Wl,--no-undefined)
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

install(TARGETS sensor_filter
			LIBRARY DESTINATION ${LIB_INSTALL_DIR}
			ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
			RUNTIME DESTINATION ${BIN_INSTALL_DIR}
			INCLUDES DESTINATION ${INCLUDE_INSTALL_DIR}
			)
install(
			FILES
			${LIB_HEADER_FILES}
			DESTINATION ${INCLUDE_INSTALL_DIR}/sensor_filter
			COMPONENT sensor_filter
	)


 
 
