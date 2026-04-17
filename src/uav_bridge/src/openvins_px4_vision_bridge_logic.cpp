#include "uav_bridge/openvins_px4_vision_bridge_logic.hpp"

#include <tf2/LinearMath/Matrix3x3.h>

namespace uav_bridge
{

tf2::Quaternion orientationEnuBodyFromEnuSensor(
  const tf2::Quaternion & orientation_enu_sensor,
  const tf2::Quaternion & body_to_sensor)
{
  tf2::Quaternion q_enu_sensor = orientation_enu_sensor;
  q_enu_sensor.normalize();

  tf2::Quaternion q_body_to_sensor = body_to_sensor;
  q_body_to_sensor.normalize();

  const tf2::Matrix3x3 rotation_enu_sensor(q_enu_sensor);
  const tf2::Matrix3x3 rotation_body_to_sensor(q_body_to_sensor);
  const tf2::Matrix3x3 rotation_enu_body = rotation_enu_sensor * rotation_body_to_sensor;

  tf2::Quaternion q_enu_body;
  rotation_enu_body.getRotation(q_enu_body);
  q_enu_body.normalize();
  return q_enu_body;
}

tf2::Vector3 velocityBodyFluFromSensor(
  const tf2::Vector3 & velocity_sensor,
  const tf2::Quaternion & body_to_sensor)
{
  tf2::Quaternion q_body_to_sensor = body_to_sensor;
  q_body_to_sensor.normalize();

  const tf2::Matrix3x3 rotation_body_to_sensor(q_body_to_sensor);
  const tf2::Matrix3x3 rotation_sensor_to_body = rotation_body_to_sensor.transpose();
  return rotation_sensor_to_body * velocity_sensor;
}

}  // namespace uav_bridge
