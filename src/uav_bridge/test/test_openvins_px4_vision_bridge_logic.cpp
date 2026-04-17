#include <gtest/gtest.h>

#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "uav_bridge/openvins_px4_vision_bridge_logic.hpp"

namespace
{

using uav_bridge::orientationEnuBodyFromEnuSensor;
using uav_bridge::velocityBodyFluFromSensor;

TEST(OpenVinsPx4VisionBridgeLogicTest, OrientationUsesBodyToSensorContract)
{
  tf2::Quaternion body_to_sensor;
  body_to_sensor.setRPY(0.0, 0.0, M_PI_2);
  body_to_sensor.normalize();

  const tf2::Quaternion orientation_enu_sensor = body_to_sensor.inverse();
  const tf2::Quaternion orientation_enu_body =
    orientationEnuBodyFromEnuSensor(orientation_enu_sensor, body_to_sensor);

  const tf2::Matrix3x3 rotation_enu_body(orientation_enu_body);
  tf2::Matrix3x3 identity;
  identity.setIdentity();

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(rotation_enu_body[row][col], identity[row][col], 1.0e-9);
    }
  }
}

TEST(OpenVinsPx4VisionBridgeLogicTest, VelocityUsesInverseOfBodyToSensor)
{
  tf2::Quaternion body_to_sensor;
  body_to_sensor.setRPY(0.0, 0.0, M_PI_2);
  body_to_sensor.normalize();

  const tf2::Vector3 velocity_sensor(0.0, 1.0, 0.0);
  const tf2::Vector3 velocity_body = velocityBodyFluFromSensor(velocity_sensor, body_to_sensor);

  EXPECT_NEAR(velocity_body.x(), 1.0, 1.0e-9);
  EXPECT_NEAR(velocity_body.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(velocity_body.z(), 0.0, 1.0e-9);
}

TEST(OpenVinsPx4VisionBridgeLogicTest, DefaultOrbbecRotationMapsSensorAxesIntoBodyFlu)
{
  tf2::Quaternion body_to_sensor;
  body_to_sensor.setRPY(0.0, -M_PI_2, M_PI_2);
  body_to_sensor.normalize();

  const tf2::Vector3 velocity_forward_sensor(0.0, 0.0, 1.0);
  const tf2::Vector3 velocity_right_sensor(1.0, 0.0, 0.0);
  const tf2::Vector3 velocity_down_sensor(0.0, 1.0, 0.0);

  const tf2::Vector3 velocity_forward_body =
    velocityBodyFluFromSensor(velocity_forward_sensor, body_to_sensor);
  const tf2::Vector3 velocity_right_body =
    velocityBodyFluFromSensor(velocity_right_sensor, body_to_sensor);
  const tf2::Vector3 velocity_down_body =
    velocityBodyFluFromSensor(velocity_down_sensor, body_to_sensor);

  EXPECT_NEAR(velocity_forward_body.x(), 1.0, 1.0e-9);
  EXPECT_NEAR(velocity_forward_body.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(velocity_forward_body.z(), 0.0, 1.0e-9);

  EXPECT_NEAR(velocity_right_body.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(velocity_right_body.y(), -1.0, 1.0e-9);
  EXPECT_NEAR(velocity_right_body.z(), 0.0, 1.0e-9);

  EXPECT_NEAR(velocity_down_body.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(velocity_down_body.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(velocity_down_body.z(), -1.0, 1.0e-9);
}

}  // namespace
