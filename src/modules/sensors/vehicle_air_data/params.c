/**
 * QNH for barometer
 *
 * @min 500
 * @max 1500
 * @group Sensors
 * @unit hPa
 */
PARAM_DEFINE_FLOAT(SENS_BARO_QNH, 1013.25f);

/**
 * Baro max rate.
 *
 * Barometric air data maximum publication rate. This is an upper bound,
 * actual barometric data rate is still dependent on the sensor.
 *
 * @min 1
 * @max 200
 * @group Sensors
 * @unit Hz
 */
PARAM_DEFINE_FLOAT(SENS_BARO_RATE, 20.0f);

/**
 * Barometer auto calibration
 *
 * Automatically calibrate barometer based on the GNSS height
 *
 * @boolean
 *
 * @category system
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_BAR_AUTOCAL, 1);
