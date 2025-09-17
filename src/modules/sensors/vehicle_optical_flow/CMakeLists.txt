px4_add_library(vehicle_optical_flow
	VehicleOpticalFlow.cpp
	VehicleOpticalFlow.hpp
)
target_link_libraries(vehicle_optical_flow PRIVATE px4_work_queue)

if(BUILD_TESTING)
	add_subdirectory(test)
endif()
