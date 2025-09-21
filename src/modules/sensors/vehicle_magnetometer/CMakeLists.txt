px4_add_library(vehicle_magnetometer
	VehicleMagnetometer.cpp
	VehicleMagnetometer.hpp
)

target_link_libraries(vehicle_magnetometer
	PRIVATE
		data_validator
		px4_work_queue
		sensor_calibration
)
