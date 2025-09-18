add_subdirectory(airframes)

px4_add_romfs_files(

	rc.autostart_ext

	rcS
	rc.sensors
	rc.vehicle_setup

	# TODO
	rc.balloon_apps
	rc.balloon_defaults
	rc.sysinit
)

if(CONFIG_MODULES_AIRSHIP_ATT_CONTROL)
	px4_add_romfs_files(
		rc.airship_apps
		rc.airship_defaults
	)
endif()

if(CONFIG_MODULES_FW_RATE_CONTROL)
	px4_add_romfs_files(
		rc.fw_apps
		rc.fw_defaults
	)
endif()

if(CONFIG_MODULES_MC_RATE_CONTROL)
	px4_add_romfs_files(
		rc.heli_defaults
		rc.mc_apps
		rc.mc_defaults
	)
endif()

if(CONFIG_MODULES_ROVER_DIFFERENTIAL)
	px4_add_romfs_files(
		rc.rover_differential_apps
		rc.rover_differential_defaults
	)
endif()

if(CONFIG_MODULES_ROVER_ACKERMANN)
	px4_add_romfs_files(
		rc.rover_ackermann_apps
		rc.rover_ackermann_defaults
	)
endif()

if(CONFIG_MODULES_SPACECRAFT)
	px4_add_romfs_files(
		rc.sc_apps
		rc.sc_defaults
	)
endif()

if(CONFIG_MODULES_ROVER_MECANUM)
	px4_add_romfs_files(
		rc.rover_mecanum_apps
		rc.rover_mecanum_defaults
	)
endif()

if(CONFIG_MODULES_UUV_ATT_CONTROL)
	px4_add_romfs_files(
		rc.uuv_apps
		rc.uuv_defaults
	)
endif()

if(CONFIG_MODULES_VTOL_ATT_CONTROL)
	px4_add_romfs_files(
		rc.vtol_apps
		rc.vtol_defaults
	)
endif()


if(CONFIG_MODULES_LOGGER)
	px4_add_romfs_files(
		rc.logging
	)
endif()

if(CONFIG_MODULES_TEMPERATURE_COMPENSATION)
	px4_add_romfs_files(
		rc.thermal_cal
	)
endif()
