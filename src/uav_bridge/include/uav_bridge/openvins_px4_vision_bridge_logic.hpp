#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace uav_bridge
{

tf2::Quaternion orientationEnuBodyFromEnuSensor(
  const tf2::Quaternion & orientation_enu_sensor,
  const tf2::Quaternion & body_to_sensor);

tf2::Vector3 velocityBodyFluFromSensor(
  const tf2::Vector3 & velocity_sensor,
  const tf2::Quaternion & body_to_sensor);

}  // namespace uav_bridge
