#ifndef __IS_MSG_ROBOT_CONTROLLER_HPP__
#define __IS_MSG_ROBOT_CONTROLLER_HPP__

#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>
#include <is/packer.hpp>

namespace is {
namespace msg {
namespace controller {

using namespace is::msg::robot;
using namespace is::msg::camera;
using namespace is::msg::geometry;
using namespace is::msg::common;
using namespace boost;

struct RobotTask {
  std::vector<Point> positions;  // <x,y> [mm]
  std::vector<Point> speeds;     // <dx,dy> [mm/s]
  double stop_distance;          // [mm]

  IS_DEFINE_MSG(positions, speeds, stop_distance)
};

struct RobotControllerStatus {
  Speed speed;
  optional<Pose> current_pose;
  optional<Pose> control_pose;
  optional<Pose> desired_pose;
  optional<double> error;
  bool arrived;

  IS_DEFINE_MSG(speed, current_pose, control_pose, desired_pose, error, arrived)
};

}  // ::controller
}  // ::msg
}  // ::is

#endif  // __IS_MSG_ROBOT_CONTROLLER_HPP__