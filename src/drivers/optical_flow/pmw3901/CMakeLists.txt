px4_add_module(
	MODULE drivers__optical_flow__pmw3901
	MAIN pmw3901
	SRCS
		pmw3901_main.cpp
		PMW3901.cpp
		PMW3901.hpp
	)
