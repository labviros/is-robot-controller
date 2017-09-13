#include "msgs/robot-controller.hpp"

#include <boost/program_options.hpp>
#include <is/is.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>

using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::geometry;
using namespace is::msg::controller;
namespace po = boost::program_options;

int main(int argc, char* argv[]) {
  std::string uri;
  std::string robot;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://edge.is:30000"), "broker uri");
  options("robot,r", po::value<std::string>(&robot)->default_value("robot.0"), "robot name");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << description << std::endl;
    return 1;
  }

  auto is = is::connect(uri);
  std::string topic = "robot-controller" + robot.substr(robot.find_last_of('.')) + ".status";
  auto tag = is.subscribe(topic);

  while (1) {
    auto envelope = is.consume(tag);
    is::log::info("NEW STATUS");
    auto status = is::msgpack<RobotControllerStatus>(envelope);
    if (status.current_pose) {
      Pose pose = *status.current_pose;
      is::log::info("Current pose: {},{},{}", pose.position.x, pose.position.y, pose.heading);
    } else {
      is::log::info("Current pose: -,-,-");
    }
    if (status.desired_pose) {
      Pose pose = *status.desired_pose;
      is::log::info("Desired pose: {},{},{}", pose.position.x, pose.position.y, pose.heading);
    } else {
      is::log::info("Desired pose: -,-,-");
    }
    is::log::info("Speed: {},{}", status.speed.linear, status.speed.angular);
    is::log::info("Arrived: {}", status.arrived);
  }
  return 0;
}