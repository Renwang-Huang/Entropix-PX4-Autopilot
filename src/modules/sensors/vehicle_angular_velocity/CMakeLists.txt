px4_add_library(vehicle_angular_velocity
	VehicleAngularVelocity.cpp
	VehicleAngularVelocity.hpp
)

target_compile_options(vehicle_angular_velocity
	PRIVATE
		${MAX_CUSTOM_OPT_LEVEL}
		#-DDEBUG_BUILD
)

target_link_libraries(vehicle_angular_velocity
	PRIVATE
		mathlib
		px4_work_queue
		sensor_calibration
)
