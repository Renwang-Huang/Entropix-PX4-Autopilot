add_subdirectory(data_validator)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(CONFIG_SENSORS_VEHICLE_ACCELERATION)
	add_subdirectory(vehicle_acceleration)
endif()

add_subdirectory(vehicle_imu)

if(CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY)
	add_subdirectory(vehicle_angular_velocity)
endif()

if(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	add_subdirectory(vehicle_magnetometer)
endif()

if(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)
	add_subdirectory(vehicle_optical_flow)
endif()

px4_add_module(
	MODULE modules__sensors
	MAIN sensors
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
	SRCS
		sensors.cpp
		voted_sensors_update.cpp
		voted_sensors_update.h
		Integrator.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		conversion
		data_validator
		mathlib
		sensor_calibration
		vehicle_imu
	)

if(CONFIG_SENSORS_VEHICLE_ACCELERATION)
	target_link_libraries(modules__sensors PRIVATE vehicle_acceleration)
endif()

if(CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY)
	target_link_libraries(modules__sensors PRIVATE vehicle_angular_velocity)
endif()

if(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	target_link_libraries(modules__sensors PRIVATE vehicle_magnetometer)
endif()

if(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)
	target_link_libraries(modules__sensors PRIVATE vehicle_optical_flow)
endif()
