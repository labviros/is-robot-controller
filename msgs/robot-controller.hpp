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

struct Trajectory {
  std::vector<Point> positions;  // <x,y> [mm]
  std::vector<Point> speeds;     // <dx,dy> [mm/s]
  double final_error;

  IS_DEFINE_MSG(positions, speeds, final_error)
};

}  // ::controller
}  // ::msg
}  // ::is

#endif  // __IS_MSG_ROBOT_CONTROLLER_HPP__