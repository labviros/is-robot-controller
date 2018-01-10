#ifndef __FENCE_HPP__
#define __FENCE_HPP__

#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include <armadillo>
#include <is/is.hpp>
#include "robot-parameters.pb.h"

namespace fence {

using namespace is::robot;
using namespace is::common;

Speed limit_speed(Speed speed, float p, float min, float max) {
  auto dp = fabs(p - max);
  auto p_out = p > max || p < min;
  if (p_out) {
    is::warn("Robot leaving critical area. Reducing speed.");
    speed.set_linear(std::max(1.0 - dp / 0.5, 0.0) * speed.linear());
    speed.set_angular(std::max(1.0 - dp / 0.5, 0.0) * speed.angular());
  }
  return speed;
}

Speed limit_speed(RobotControllerProgress const& status, Fence const& fence) {
  Speed speed;
  if (!status.has_current_pose())
    return speed;
  speed = limit_speed(status.current_speed(), status.current_pose().position().x(),  fence.x_min(), fence.x_max());
  return limit_speed(speed, status.current_pose().position().y(),  fence.y_min(), fence.y_max());
}

}  // ::fence

#endif  // __FENCE_HPP__