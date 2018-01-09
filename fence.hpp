#ifndef __FENCE_HPP__
#define __FENCE_HPP__

#include <is/is.hpp>
#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include "robot-parameters.pb.h"
#include <armadillo>

namespace fence {

using namespace is::robot;
using namespace is::common;

Speed limit_speed(RobotControllerProgress const& status, Fence const& fence) {
  Speed speed;
  if (!status.has_current_pose())
    return speed;
  auto x = status.current_pose().position().x();
  auto y = status.current_pose().position().y();
  auto x_out = x > fence.x_max() || x < fence.x_min();
  auto y_out = y > fence.y_max() || y < fence.y_min();
  if (x_out || y_out) {
    is::warn("Robot leaving critical area. Forcing stop.");
    return speed;
  }
  return status.current_speed();
}

}  // ::fence

#endif  // __FENCE_HPP__