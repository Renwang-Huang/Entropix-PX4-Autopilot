add_subdirectory(airframes)

px4_add_romfs_files(

	rc.autostart_ext

	rcS
	rc.sensors
	rc.vehicle_setup

	rc.balloon_apps
	rc.balloon_defaults
	rc.sysinit
)

px4_add_romfs_files(
	rc.heli_defaults
	rc.mc_apps
	rc.mc_defaults
)
