#pragma once

#include "data_validator/DataValidatorGroup.hpp"
#include "RingBuffer.hpp"

#include <Integrator.hpp>

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_optical_flow.h>
#include <uORB/topics/vehicle_optical_flow_vel.h>

namespace sensors
{

class VehicleOpticalFlow : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleOpticalFlow();
	~VehicleOpticalFlow() override;

	bool Start();
	void Stop();

	void PrintStatus();

protected:
	void UpdateDistanceSensor();
	int _distance_sensor_selected{-1}; //记录当前选择的距离传感器实例编号（因为可能有多个距离传感器）

private:
	void ClearAccumulatedData();
	void UpdateSensorGyro();

	void Run() override;

	void ParametersUpdate();
	void SensorCorrectionsUpdate(bool force = false);//更新传感器标定/修正

	static constexpr int MAX_SENSOR_COUNT = 3;//最多支持三个光流或距离传感器实例

	uORB::Publication<vehicle_optical_flow_s> _vehicle_optical_flow_pub{ORB_ID(vehicle_optical_flow)};//发布完整光流数据，光流“全貌”，包含位移、时间、质量等完整信息
	uORB::Publication<vehicle_optical_flow_vel_s> _vehicle_optical_flow_vel_pub{ORB_ID(vehicle_optical_flow_vel)};//发布光流速度信息，光流“提炼版”，只提供平面速度

	uORB::Subscription _params_sub{ORB_ID(parameter_update)};

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};//订阅多路距离传感器的数据（不论ID多少都进行订阅）

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};//飞机姿态信息

	uORB::SubscriptionCallbackWorkItem _sensor_flow_sub{this, ORB_ID(sensor_optical_flow)};//光流原始数据
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub{this, ORB_ID(sensor_gyro)};//陀螺仪数据
	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};//选择指定ID的传感器数据

	sensors::IntegratorConing _gyro_integrator{};

	hrt_abstime _gyro_timestamp_sample_last{0};

	calibration::Gyroscope _gyro_calibration{};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	matrix::Dcmf _flow_rotation{matrix::eye<float, 3>()};

	hrt_abstime _flow_timestamp_sample_last{0};
	matrix::Vector2f _flow_integral{};
	matrix::Vector3f _delta_angle{};
	uint32_t _integration_timespan_us{};
	float _distance_sum{NAN};
	uint8_t _distance_sum_count{0};
	uint16_t _quality_sum{0};
	uint8_t _accumulated_count{0};

	hrt_abstime _last_range_sensor_update{0};

	bool _delta_angle_available{false};

	//gyroSample结构体定义
	struct gyroSample {
		uint64_t time_us{}; ///< timestamp of the measurement (uSec)
		matrix::Vector3f data{};
		float dt{0.f};
	};

	struct rangeSample {
		uint64_t time_us{}; ///< timestamp of the measurement (uSec)
		float data{};
	};

	RingBuffer<gyroSample, 32> _gyro_buffer{};//_gyro_buffer.push(gyro_sample);//压入环形缓冲区，供光流融合使用
	RingBuffer<rangeSample, 5> _range_buffer{};//_range_buffer.push(sample);//将数据封装成 rangeSample 并推入 _range_buffer

	//PX4参数系统，用于在地面站设置光流模块参数
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_FLOW_ROT>) _param_sens_flow_rot,
		//以45°为步进，顺时针旋转，定义光流模块的安装方位（机体坐标系定义为X前Y右Z下，默认0°表示光流模块朝前，和机体朝向一致）
		(ParamFloat<px4::params::SENS_FLOW_MINHGT>) _param_sens_flow_minhgt,
		//定义了光流传感器在依赖光流进行定位时的最小有效高度（参数范围：0.0 ~ 1.0 m，步进0.1m，默认值0.08m，表示默认光流在低于8cm的时候不可靠）
		(ParamFloat<px4::params::SENS_FLOW_MAXHGT>) _param_sens_flow_maxhgt,
		//这是光流可靠工作的最大高度（默认值是100m，超过这个高度，光流传感器仍然可能有数据，但精度会快速下降）
		(ParamFloat<px4::params::SENS_FLOW_MAXR>) _param_sens_flow_maxr,
		//光流传感器能可靠测量的最大角速度，单位是rad/s，光流传感器在速度过大时，输出的数据已经不再准确，SENS_FLOW_MAXR参数设置安全范围，超过就丢弃光流数据，避免误导状态估计
		(ParamFloat<px4::params::SENS_FLOW_RATE>) _param_sens_flow_rate,
		//光流数据的最大发布频率（单位 Hz），如果传感器刷新是100HZ但是这个参数设置为70HZ，那么系统中该消息仍然是70HZ的刷新率，如果传感器刷新是70HZ但是这个参数设置为100HZ，那么系统中该消息仍然是70HZ刷新率
		(ParamFloat<px4::params::SENS_FLOW_SCALE>) _param_sens_flow_scale
		//对光流测量值进行线性放大或缩小，光流传感器测得的位移或速度量值会乘以这个参数再发布给系统，如果光流测得水平速度为0.2 m/s，SENS_FLOW_SCALE=1.2 → 系统看到的是0.24m/s，用于修正光流传感器的标定偏差，可以微调飞控系统对光流的响应灵敏度
	)
};
};
