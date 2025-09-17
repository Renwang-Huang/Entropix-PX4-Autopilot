//这个文件是光流模块参数的定义文件

/**
 * Optical flow rotation
 *
 * This parameter defines the yaw rotation of the optical flow relative to the vehicle body frame.
 * Zero rotation is defined as X on flow board pointing towards front of vehicle.
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 *
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_FLOW_ROT, 0);

/**
 * Minimum height above ground when reliant on optical flow.
 *
 * This parameter defines the minimum distance from ground at which the optical flow sensor operates reliably.
 * The sensor may be usable below this height, but accuracy will progressively reduce to loss of focus.
 *
 * @unit m
 * @min 0.0
 * @max 1.0
 * @increment 0.1
 * @decimal 2
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_FLOW_MINHGT, 0.08f);

/**
 * Maximum height above ground when reliant on optical flow.
 *
 * This parameter defines the maximum distance from ground at which the optical flow sensor operates reliably.
 * The height setpoint will be limited to be no greater than this value when the navigation system
 * is completely reliant on optical flow data and the height above ground estimate is valid.
 * The sensor may be usable above this height, but accuracy will progressively degrade.
 *
 * @unit m
 * @min 1.0
 * @max 100.0
 * @increment 0.1
 * @decimal 2
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_FLOW_MAXHGT, 100.f);

/**
 * Magnitude of maximum angular flow rate reliably measurable by the optical flow sensor.
 *
 * Optical flow data will not fused by the estimators if the magnitude of the flow rate exceeds this value and
 * control loops will be instructed to limit ground speed such that the flow rate produced by movement over ground
 * is less than 50% of this value.
 *
 * @unit rad/s
 * @min 1.0
 * @decimal 2
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_FLOW_MAXR, 8.f);

/**
 * Optical flow max rate.
 *
 * Optical flow data maximum publication rate. This is an upper bound,
 * actual optical flow data rate is still dependent on the sensor.
 *
 * @min 1
 * @max 200
 * @group Sensors
 * @unit Hz
 *
 * @reboot_required true
 *
 */
PARAM_DEFINE_FLOAT(SENS_FLOW_RATE, 70.0f);

/**
 * Optical flow scale factor
 *
 * @min 0.5
 * @max 1.5
 * @decimal 2
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SENS_FLOW_SCALE, 1.f);
