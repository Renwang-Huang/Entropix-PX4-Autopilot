//测试以确保光流数据一定来自于下视觉传感器（而不是前视觉传感器等）

//意味着PX4只支持下视觉传感器而不支持其他方向的视觉传感器（光流模块）

#include <gtest/gtest.h>
#include "../VehicleOpticalFlow.hpp"
#include <uORB/uORBManager.hpp>
#include <uORB/topics/distance_sensor.h>


distance_sensor_s createDistanceSensorMessage(uint16_t orientation)
{
	distance_sensor_s message;
	message.timestamp = hrt_absolute_time();
	message.min_distance = 1.f;
	message.max_distance = 100.f;
	message.current_distance = 1.1f;

	message.variance = 0.1f;
	message.signal_quality = 100;
	message.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	message.orientation = orientation;
	message.h_fov = math::radians(50.f);
	message.v_fov = math::radians(30.f);
	return message;

}

class VehicleOpticalFlowTest : public ::testing::Test
{
public:

	class VehicleOpticalFlowTestable  : public  sensors::VehicleOpticalFlow
	{
	public:
		void UpdateDistanceSensorPublic()
		{
			VehicleOpticalFlow::UpdateDistanceSensor();
		}
		bool IsDistanceSensorSelected()
		{
			return _distance_sensor_selected >= 0;

		}
	};

	void SetUp() override
	{
		uORB::Manager::initialize();

	}
	void TearDown() override
	{
		uORB::Manager::terminate();
	}
};


TEST_F(VehicleOpticalFlowTest, CameraFacingDown)
{
	// GIVEN: message with sensor camera facing down
	distance_sensor_s message = createDistanceSensorMessage(distance_sensor_s::ROTATION_DOWNWARD_FACING);
	orb_advertise(ORB_ID(distance_sensor), &message);

	// WHEN: update distance sensor
	VehicleOpticalFlowTest::VehicleOpticalFlowTestable testable;
	testable.UpdateDistanceSensorPublic();

	// THEN: sensor selected
	EXPECT_TRUE(testable.IsDistanceSensorSelected());
}

TEST_F(VehicleOpticalFlowTest, CameraFacingForward)
{
	// GIVEN: message with sensor camera facing forward
	distance_sensor_s message = createDistanceSensorMessage(distance_sensor_s::ROTATION_FORWARD_FACING);
	orb_advertise(ORB_ID(distance_sensor), &message);

	// WHEN: update distance sensor
	VehicleOpticalFlowTest::VehicleOpticalFlowTestable testable;
	testable.UpdateDistanceSensorPublic();

	// THEN: sensor is not selected
	EXPECT_FALSE(testable.IsDistanceSensorSelected());
}
