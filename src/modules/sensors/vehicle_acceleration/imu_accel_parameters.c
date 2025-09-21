/**
* Low pass filter cutoff frequency for accel
*
* The cutoff frequency for the 2nd order butterworth filter on the primary accelerometer.
* This only affects the signal sent to the controllers, not the estimators. 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Sensors
*/
PARAM_DEFINE_FLOAT(IMU_ACCEL_CUTOFF, 30.0f);
