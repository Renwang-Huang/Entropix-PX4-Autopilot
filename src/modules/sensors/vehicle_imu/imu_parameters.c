/**
* IMU integration rate.
*
* The rate at which raw IMU data is integrated to produce delta angles and delta velocities.
* Recommended to set this to a multiple of the estimator update period (currently 10 ms for ekf2).
*
* @min 100
* @max 1000
* @value 100 100 Hz
* @value 200 200 Hz
* @value 250 250 Hz
* @value 400 400 Hz
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_INT32(IMU_INTEG_RATE, 200);

/**
 * IMU auto calibration
 *
 * Automatically initialize IMU (accel/gyro) calibration from bias estimates if available.
 *
 * @boolean
 *
 * @category system
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_IMU_AUTOCAL, 1);

/**
 * IMU notify clipping
 *
 * Notify the user if the IMU is clipping
 *
 * @boolean
 *
 * @category system
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_IMU_CLPNOTI, 1);
