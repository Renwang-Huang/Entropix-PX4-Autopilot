#include "VehicleOpticalFlow.hpp"

#include <px4_platform_common/log.h>

namespace sensors
{

using namespace matrix;
using namespace time_literals;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

//初始化相关模块
VehicleOpticalFlow::VehicleOpticalFlow() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_optical_flow_pub.advertise();

	_gyro_integrator.set_reset_samples(1);
}

//停止光流处理任务
VehicleOpticalFlow::~VehicleOpticalFlow()
{
	Stop();
	perf_free(_cycle_perf);
}

//触发传感器回调，启动对应的数据处理
bool VehicleOpticalFlow::Start()
{
	_sensor_flow_sub.registerCallback();

	_sensor_gyro_sub.registerCallback();
	_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH / 2);

	_sensor_selection_sub.registerCallback();

	ScheduleNow();
	return true;
}

//停止模块，取消订阅与清理状态
void VehicleOpticalFlow::Stop()
{
	Deinit();

	_sensor_flow_sub.unregisterCallback();
	_sensor_gyro_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();
}

//在运行过程中动态检查参数是否变化，并更新模块内部状态
void VehicleOpticalFlow::ParametersUpdate()
{
	if (_params_sub.updated()) {
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_flow_rotation = get_rot_matrix((enum Rotation)_param_sens_flow_rot.get());
	}
}

void VehicleOpticalFlow::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	UpdateDistanceSensor();

	if (!_delta_angle_available) {
		UpdateSensorGyro();   //如果光流传感器本身没有提供角速度积分（delta_angle），就使用陀螺仪数据进行补充
	}

	//存储当前获取的光流原始数据
	sensor_optical_flow_s sensor_optical_flow;

	if (_sensor_flow_sub.update(&sensor_optical_flow)) {

		// 如果有数据间隙，或者质量计数异常，调用 ClearAccumulatedData() 重置累积量
		const uint64_t integration_gap_threshold_us = sensor_optical_flow.integration_timespan_us * 2;

		if ((sensor_optical_flow.timestamp_sample >= _flow_timestamp_sample_last + integration_gap_threshold_us)
		    || (_accumulated_count > 0 && (sensor_optical_flow.quality > 0) && _quality_sum == 0)) {

			ClearAccumulatedData();
		}


		const hrt_abstime timestamp_oldest = sensor_optical_flow.timestamp_sample - sensor_optical_flow.integration_timespan_us;
		const hrt_abstime timestamp_newest = sensor_optical_flow.timestamp;

		// （角速度积分）处理
		if (sensor_optical_flow.delta_angle_available && Vector2f(sensor_optical_flow.delta_angle).isAllFinite()) {
			Vector3f delta_angle(sensor_optical_flow.delta_angle);

			if (!PX4_ISFINITE(delta_angle(2))) {   //如果传感器只提供XY，Z轴置NAN
				delta_angle(2) = 0.f;
				_delta_angle += _flow_rotation * delta_angle;
				_delta_angle(2) = NAN;

			} else {
				_delta_angle += _flow_rotation * delta_angle;
			}

			_delta_angle_available = true;

		} else {
			_delta_angle_available = false;//如果光流传感器没有提供 delta_angle

			gyroSample gyro_sample;//结构体都有在hpp文件中定义

			//这里pop_oldest调用了RingBuffer.hpp里面的方法
			while (_gyro_buffer.pop_oldest(timestamp_oldest, timestamp_newest, &gyro_sample)) {

				_gyro_integrator.put(gyro_sample.data, gyro_sample.dt);

				float min_interval_s = (sensor_optical_flow.integration_timespan_us * 1e-6f) * 0.99f;

				if (_gyro_integrator.integral_dt() > min_interval_s) {
					PX4_INFO("integral dt: %.6f, min interval: %.6f", (double)_gyro_integrator.integral_dt(),(double) min_interval_s);
					break;
				}
			}

			Vector3f delta_angle{NAN, NAN, NAN};
			uint32_t delta_angle_dt;

			if (_gyro_integrator.reset(delta_angle, delta_angle_dt)) {
				_delta_angle += delta_angle;

			} else {
				// force integrator reset
				_gyro_integrator.reset();
			}
		}

		// 高度数据处理
		if (sensor_optical_flow.distance_available && PX4_ISFINITE(sensor_optical_flow.distance_m)) {
			if (!PX4_ISFINITE(_distance_sum)) {
				_distance_sum = sensor_optical_flow.distance_m;
				_distance_sum_count = 1;

			} else {
				_distance_sum += sensor_optical_flow.distance_m;
				_distance_sum_count += 1;
			}

		} else {
			rangeSample range_sample;

			if (_range_buffer.peak_first_older_than(sensor_optical_flow.timestamp_sample, &range_sample)) {
				if (!PX4_ISFINITE(_distance_sum)) {
					_distance_sum = range_sample.data;
					_distance_sum_count = 1;

				} else {
					_distance_sum += range_sample.data;
					_distance_sum_count += 1;
				}
			}
		}

		_flow_timestamp_sample_last = sensor_optical_flow.timestamp_sample;

		//_flow_integral：累积像素位移（光流测量）
		_flow_integral(0) += sensor_optical_flow.pixel_flow[0];
		_flow_integral(1) += sensor_optical_flow.pixel_flow[1];

		//_integration_timespan_us：累积积分时间
		_integration_timespan_us += sensor_optical_flow.integration_timespan_us;

		//_quality_sum：累积光流质量
		_quality_sum += sensor_optical_flow.quality;

		//_accumulated_count：累积次数
		_accumulated_count++;

		bool publish = true;

		if (_param_sens_flow_rate.get() > 0) {
			const float interval_us = 1e6f / _param_sens_flow_rate.get();

			// SENS_FLOW_RATE 参数作用的地方
			if (_integration_timespan_us < interval_us) {
				publish = false;
			}
		}

		if (publish) {
			vehicle_optical_flow_s vehicle_optical_flow{};

			vehicle_optical_flow.timestamp_sample = sensor_optical_flow.timestamp_sample;
			vehicle_optical_flow.device_id = sensor_optical_flow.device_id;

			_flow_integral *= _param_sens_flow_scale.get();
			_flow_integral.copyTo(vehicle_optical_flow.pixel_flow);
			_delta_angle.copyTo(vehicle_optical_flow.delta_angle);

			vehicle_optical_flow.integration_timespan_us = _integration_timespan_us;

			vehicle_optical_flow.quality = _quality_sum / _accumulated_count;

			if (_distance_sum_count > 0 && PX4_ISFINITE(_distance_sum)) {
				vehicle_optical_flow.distance_m = _distance_sum / _distance_sum_count;

			} else {
				vehicle_optical_flow.distance_m = NAN;
			}

			// SENS_FLOW_MAXR
			if (PX4_ISFINITE(sensor_optical_flow.max_flow_rate)
			    && (sensor_optical_flow.max_flow_rate <= _param_sens_flow_maxr.get())) {

				vehicle_optical_flow.max_flow_rate = sensor_optical_flow.max_flow_rate;

			} else {
				vehicle_optical_flow.max_flow_rate = _param_sens_flow_maxr.get();
			}

			// SENS_FLOW_MINHGT
			if (PX4_ISFINITE(sensor_optical_flow.min_ground_distance)
			    && (sensor_optical_flow.min_ground_distance >= _param_sens_flow_minhgt.get())) {

				vehicle_optical_flow.min_ground_distance = sensor_optical_flow.min_ground_distance;

			} else {
				vehicle_optical_flow.min_ground_distance = _param_sens_flow_minhgt.get();
			}

			// SENS_FLOW_MAXHGT
			if (PX4_ISFINITE(sensor_optical_flow.max_ground_distance)
			    && (sensor_optical_flow.max_ground_distance <= _param_sens_flow_maxhgt.get())) {

				vehicle_optical_flow.max_ground_distance = sensor_optical_flow.max_ground_distance;

			} else {
				vehicle_optical_flow.max_ground_distance = _param_sens_flow_maxhgt.get();
			}


			// rotate (SENS_FLOW_ROT)
			float zeroval = 0.f;
			rotate_3f((enum Rotation)_param_sens_flow_rot.get(), vehicle_optical_flow.pixel_flow[0],
				  vehicle_optical_flow.pixel_flow[1], zeroval);

			vehicle_optical_flow.timestamp = hrt_absolute_time();
			_vehicle_optical_flow_pub.publish(vehicle_optical_flow);

			// vehicle_optical_flow_vel if distance is available (for logging)
			if (_distance_sum_count > 0 && PX4_ISFINITE(_distance_sum)) {
				const float range = _distance_sum / _distance_sum_count;

				vehicle_optical_flow_vel_s flow_vel{};

				flow_vel.timestamp_sample = vehicle_optical_flow.timestamp_sample;

				// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
				// is produced by a RH rotation of the image about the sensor axis.
				const Vector2f flow_xy_rad{-vehicle_optical_flow.pixel_flow[0], -vehicle_optical_flow.pixel_flow[1]};
				const Vector3f gyro_rate_integral{-vehicle_optical_flow.delta_angle[0], -vehicle_optical_flow.delta_angle[1], -vehicle_optical_flow.delta_angle[2]};

				const float flow_dt = 1e-6f * vehicle_optical_flow.integration_timespan_us;

				// compensate for body motion to give a LOS rate
				const Vector2f flow_compensated_XY_rad = flow_xy_rad - gyro_rate_integral.xy();

				Vector3f vel_optflow_body;
				vel_optflow_body(0) = - range * flow_compensated_XY_rad(1) / flow_dt;
				vel_optflow_body(1) =   range * flow_compensated_XY_rad(0) / flow_dt;
				vel_optflow_body(2) = 0.f;

				// vel_body
				flow_vel.vel_body[0] = vel_optflow_body(0);
				flow_vel.vel_body[1] = vel_optflow_body(1);

				// vel_ne
				flow_vel.vel_ne[0] = NAN;
				flow_vel.vel_ne[1] = NAN;

				vehicle_attitude_s vehicle_attitude{};

				if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
					const matrix::Dcmf R_to_earth = matrix::Quatf(vehicle_attitude.q);
					const Vector3f flow_vel_ne = R_to_earth * vel_optflow_body;

					flow_vel.vel_ne[0] = flow_vel_ne(0);
					flow_vel.vel_ne[1] = flow_vel_ne(1);
				}

				const Vector2f flow_rate(flow_xy_rad * (1.f / flow_dt));
				flow_rate.copyTo(flow_vel.flow_rate_uncompensated);

				const Vector2f flow_rate_compensated(flow_compensated_XY_rad * (1.f / flow_dt));
				flow_rate_compensated.copyTo(flow_vel.flow_rate_compensated);

				const Vector3f measured_body_rate(gyro_rate_integral * (1.f / flow_dt));

				// gyro_rate
				flow_vel.gyro_rate[0] = measured_body_rate(0);
				flow_vel.gyro_rate[1] = measured_body_rate(1);
				flow_vel.gyro_rate[2] = measured_body_rate(2);

				flow_vel.timestamp = hrt_absolute_time();

				_vehicle_optical_flow_vel_pub.publish(flow_vel);
			}

			ClearAccumulatedData();
		}
	}

	// 10ms调度一次
	ScheduleDelayed(10_ms);

	perf_end(_cycle_perf);
}

void VehicleOpticalFlow::UpdateDistanceSensor()
{
	//存储当前读取的距离传感器数据
	distance_sensor_s distance_sensor;

	//传感器的选择逻辑
	if ((_distance_sensor_selected < 0) && _distance_sensor_subs.advertised()) {
		for (unsigned i = 0; i < _distance_sensor_subs.size(); i++) {

			if (_distance_sensor_subs[i].update(&distance_sensor)) {
				if ((hrt_elapsed_time(&distance_sensor.timestamp) < 100_ms)
				    && (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {

					int ndist = orb_group_count(ORB_ID(distance_sensor));

					if (ndist > 1) {
						PX4_INFO("selected distance_sensor:%d (%d advertised)", i, ndist);
					}

					_distance_sensor_selected = i;
					_last_range_sensor_update = distance_sensor.timestamp;
					break;
				}
			}
		}
	}

	if (_distance_sensor_selected >= 0 && _distance_sensor_subs[_distance_sensor_selected].update(&distance_sensor)) {
		//传感器方向必须要朝下
		if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {

			if ((distance_sensor.current_distance >= distance_sensor.min_distance)
			    && (distance_sensor.current_distance <= distance_sensor.max_distance)) {   //测量距离必须在有效范围内

				rangeSample sample;
				sample.time_us = distance_sensor.timestamp;
				sample.data = distance_sensor.current_distance;

				_range_buffer.push(sample);//将数据封装成 rangeSample 并推入 _range_buffer

				_last_range_sensor_update = distance_sensor.timestamp;

				return;
			}

		} else {
			_distance_sensor_selected = -1;
		}
	}

	if (hrt_elapsed_time(&_last_range_sensor_update) > 1_s) {
		_distance_sensor_selected = -1;
	}
}

//不处理陀螺仪数据而是获取陀螺仪数据
void VehicleOpticalFlow::UpdateSensorGyro()
{
	if (_sensor_selection_sub.updated()) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {   //最多支持三个光流或距离传感器实例
			uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

			if (sensor_gyro_sub.advertised()   //数据有效的前提条件
			    && (sensor_gyro_sub.get().timestamp != 0)
			    && (sensor_gyro_sub.get().device_id != 0)
			    && (hrt_elapsed_time(&sensor_gyro_sub.get().timestamp) < 1_s)) {

				if (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id) {   //检查是否是用户指定的设备
					if (_sensor_gyro_sub.ChangeInstance(i) && _sensor_gyro_sub.registerCallback()) {

						_gyro_calibration.set_device_id(sensor_gyro_sub.get().device_id);
						PX4_DEBUG("selecting sensor_gyro:%" PRIu8 " %" PRIu32, i, sensor_gyro_sub.get().device_id);
						break;   //检测到有效的陀螺仪之后就停止搜索

					} else {
						PX4_ERR("unable to register callback for sensor_gyro:%" PRIu8 " %" PRIu32, i, sensor_gyro_sub.get().device_id);
					}
				}
			}
		}
	}

	bool sensor_gyro_lost_printed = false;   //打印陀螺仪数据不连续错误的标志位
	int gyro_updates = 0;

	//读取并缓存陀螺仪数据
	while (_sensor_gyro_sub.updated() && (gyro_updates < sensor_gyro_s::ORB_QUEUE_LENGTH)) {
		gyro_updates++;
		const unsigned last_generation = _sensor_gyro_sub.get_last_generation();
		sensor_gyro_s sensor_gyro;

		if (_sensor_gyro_sub.copy(&sensor_gyro)) {

			if (_sensor_gyro_sub.get_last_generation() != last_generation + 1) {
				if (!sensor_gyro_lost_printed) {
					PX4_ERR("sensor_gyro lost, generation %u -> %u", last_generation, _sensor_gyro_sub.get_last_generation());
					sensor_gyro_lost_printed = true;
				}
			}

			//应用陀螺仪校准
			_gyro_calibration.set_device_id(sensor_gyro.device_id);
			_gyro_calibration.SensorCorrectionsUpdate();

			//计算两次陀螺仪样本时间差
			const float dt_s = (sensor_gyro.timestamp_sample - _gyro_timestamp_sample_last) * 1e-6f;
			_gyro_timestamp_sample_last = sensor_gyro.timestamp_sample;

                        //创建缓冲数据并压入RingBuffer
			gyroSample gyro_sample;
			gyro_sample.time_us = sensor_gyro.timestamp_sample;
			gyro_sample.data = _gyro_calibration.Correct(Vector3f{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z});
			gyro_sample.dt = dt_s;

			_gyro_buffer.push(gyro_sample);//压入环形缓冲区，供光流融合使用
		}
	}
}

//清空光流模块中累计的测量数据，为下一次积分或新一轮测量做准备
void VehicleOpticalFlow::ClearAccumulatedData()
{
	_flow_integral.zero();//累计光流位移，存储光流在x和y方向的像素位移积分值
	_integration_timespan_us = 0;//累计光流积分的时间跨度（微秒）

	_delta_angle.zero();//累积陀螺仪角增量（roll, pitch, yaw）

	_distance_sum = NAN;//累计的距离传感器测量总和
	_distance_sum_count = 0;//累计有效测量的次数

	_quality_sum = 0;//光流质量值累积（例如亮度纹理可靠性、像素匹配质量）
	_accumulated_count = 0;//累计的光流样本数量

	_gyro_integrator.reset();//陀螺仪积分器对象，用于累积角速度得到角增量
}

void VehicleOpticalFlow::PrintStatus()
{

}

};
