px4_add_library(vehicle_imu
	VehicleIMU.cpp
	VehicleIMU.hpp
)

target_compile_options(vehicle_imu
	PRIVATE
		${MAX_CUSTOM_OPT_LEVEL}
		#-DDEBUG_BUILD
)

target_link_libraries(vehicle_imu PRIVATE px4_work_queue sensor_calibration)
