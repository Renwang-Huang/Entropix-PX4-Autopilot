px4_add_library(vehicle_acceleration
	VehicleAcceleration.cpp
	VehicleAcceleration.hpp
)

target_compile_options(vehicle_acceleration PRIVATE ${MAX_CUSTOM_OPT_LEVEL})

target_link_libraries(vehicle_acceleration
	PRIVATE
		mathlib
		px4_work_queue
		sensor_calibration
)
