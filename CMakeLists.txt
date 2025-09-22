cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE FILEPATH "PX4 source directory" FORCE)
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE FILEPATH "PX4 binary directory" FORCE)

list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/cmake)

set(PX4_GIT_TAG "Entropix_1.16.0")
set(PX4_VERSION_MAJOR 1)
set(PX4_VERSION_MINOR 16)
set(PX4_VERSION_PATCH 0)
set(PX4_GIT_HASH "Entropix")

define_property(GLOBAL PROPERTY PX4_MODULE_LIBRARIES
                 BRIEF_DOCS "PX4 module libs"
                 FULL_DOCS "List of all PX4 module libraries"
                 )

define_property(GLOBAL PROPERTY PX4_KERNEL_MODULE_LIBRARIES
                 BRIEF_DOCS "PX4 kernel side module libs"
                 FULL_DOCS "List of all PX4 kernel module libraries"
                 )

define_property(GLOBAL PROPERTY PX4_MODULE_PATHS
                 BRIEF_DOCS "PX4 module paths"
                 FULL_DOCS "List of paths to all PX4 modules"
                 )

define_property(GLOBAL PROPERTY PX4_SRC_FILES
                 BRIEF_DOCS "src files from all PX4 modules & libs"
                 FULL_DOCS "SRC files from px4_add_{module,library}"
                 )

include(px4_add_module)

set(config_module_list)
set(config_kernel_list)

find_package(PythonInterp 3)

include(px4_config)
include(kconfig)
message(STATUS "PX4 config: ${PX4_CONFIG}")
message(STATUS "PX4 platform: ${PX4_PLATFORM}")

set_property(GLOBAL PROPERTY PX4_MODULE_CONFIG_FILES)

include(platforms/${PX4_PLATFORM}/cmake/px4_impl_os.cmake)
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake)

if(EXISTS "${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake/init.cmake")
	include(init)
endif()

project(Entropix_1.16.0 CXX C ASM)

set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING "Build type" FORCE)

set(MAX_CUSTOM_OPT_LEVEL -Os)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PX4_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PX4_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PX4_BINARY_DIR})

include(ccache)

px4_os_determine_build_chip()

if(NOT PX4_CHIP_MANUFACTURER)
	message(FATAL_ERROR "px4_os_determine_build_chip() needs to set PX4_CHIP_MANUFACTURER")
endif()

if(NOT PX4_CHIP)
	message(FATAL_ERROR "px4_os_determine_build_chip() needs to set PX4_CHIP")
endif()

include(px4_add_common_flags)
px4_add_common_flags()
px4_os_add_flags()

if(EXISTS ${PX4_BOARD_DIR}/cmake/init.cmake)
	include(${PX4_BOARD_DIR}/cmake/init.cmake)
endif()

include(px4_metadata)

add_subdirectory(msg EXCLUDE_FROM_ALL)

px4_generate_airframes_xml(BOARD ${PX4_BOARD})

add_library(parameters_interface INTERFACE)
add_library(kernel_parameters_interface INTERFACE)
add_library(events_interface INTERFACE)
add_library(kernel_events_interface INTERFACE)

include(px4_add_library)
add_subdirectory(src/lib EXCLUDE_FROM_ALL)

add_subdirectory(platforms/${PX4_PLATFORM}/src/px4)
add_subdirectory(platforms EXCLUDE_FROM_ALL)

if(EXISTS "${PX4_BOARD_DIR}/CMakeLists.txt")
	add_subdirectory(${PX4_BOARD_DIR})
endif()

foreach(module ${config_module_list})
	add_subdirectory(src/${module})
endforeach()

add_subdirectory(src/lib/events EXCLUDE_FROM_ALL)

add_subdirectory(src/lib/metadata EXCLUDE_FROM_ALL)

add_subdirectory(src/lib/parameters EXCLUDE_FROM_ALL)

if(${PX4_PLATFORM} STREQUAL "nuttx" AND NOT CONFIG_BUILD_FLAT)
	target_link_libraries(parameters_interface INTERFACE usr_parameters)
	target_link_libraries(kernel_parameters_interface INTERFACE parameters)
	target_link_libraries(events_interface INTERFACE usr_events)
	target_link_libraries(kernel_events_interface INTERFACE events)
else()
	target_link_libraries(parameters_interface INTERFACE parameters)
	target_link_libraries(events_interface INTERFACE events)
endif()

add_subdirectory(platforms/${PX4_PLATFORM})

set(uorb_graph_config ${PX4_BOARD})

set(graph_module_list "")
foreach(module ${config_module_list})
	set(graph_module_list "${graph_module_list}" "--src-path" "src/${module}")
endforeach()

add_custom_command(OUTPUT ${uorb_graph_config}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/uorb_graph/create.py
		${graph_module_list} --src-path src/lib
		--merge-depends
		--exclude-path src/examples
		--exclude-path src/lib/parameters
		--file ${PX4_SOURCE_DIR}/Tools/uorb_graph/graph_${uorb_graph_config}
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	COMMENT "Generating uORB graph"
)
add_custom_target(uorb_graph DEPENDS ${uorb_graph_config})

include(bloaty)
include(metadata)
include(package)

add_custom_target(install_python_requirements
	COMMAND ${PYTHON_EXECUTABLE} -m pip install --break-system-packages --requirement ${PX4_SOURCE_DIR}/Tools/setup/requirements.txt
	DEPENDS ${PX4_SOURCE_DIR}/Tools/setup/requirements.txt
	USES_TERMINAL
)

if(EXISTS "${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake/finalize.cmake")
	include(finalize)
endif()
