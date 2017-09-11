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
  std::string topic;
  std::string type;
  double rate;
  double desired_x;
  double desired_y;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://edge.is:30000"), "broker uri");
  options("rate,R", po::value<double>(&rate)->default_value(5.0), "sampling rate");
  options("topic,T", po::value<std::string>(&topic)->default_value("robot-controller.0.do-task"),
          "topic to request robot task");
  options("type,t", po::value<std::string>(&type)->default_value("none"),
          "Task type [none/final_position/trajectory/path]");
  options("desired_x,x", po::value<double>(&desired_x)->default_value(0.0),
          "Desired x pose. Use with 'final_position' type.");
  options("desired_y,y", po::value<double>(&desired_y)->default_value(0.0),
          "Desired y pose. Use with 'final_position' type.");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << description << std::endl;
    return 1;
  }

  RobotTask robot_task;

  if (type == "final_position") {
    Point p;
    p.x = desired_x;
    p.y = desired_y;
    robot_task.positions.push_back(p);
    robot_task.stop_distance = 100.0;

  } else if (type == "trajectory") {
    const double R = 900.0;      // [mm]
    const double w = 250.0 / R;  // lx => max_vel
    auto positions = [&](double t) {
      Point p;
      p.x = R * cos(w * t);
      p.y = R * sin(w * t);
      return p;
    };
    auto speeds = [&](double t) {
      Point p;
      p.x = -w * R * sin(w * t);
      p.y = w * R * cos(w * t);
      return p;
    };

    for (int t = 0; t < 40 * rate; ++t) {
      robot_task.positions.push_back(positions(t / rate));
      robot_task.speeds.push_back(speeds(t / rate));
    }
    robot_task.stop_distance = 200.0;

	} else if (type == "path") {
    const double R = 900.0;                 // [mm]
    const double w = std::atan(1.0) / 2.0;  // lx => max_vel
    auto positions = [&](double t) {
      Point p;
      p.x = R * cos(w * t);
      p.y = R * sin(w * t);
      return p;
    };
    for (int t = 0; t < 16; ++t) {
      robot_task.positions.push_back(positions(t));
    }
    robot_task.stop_distance = 200.0;
  }

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  client.request(topic, is::msgpack(robot_task));
  is::log::info("Task '{}' requested", type);

  return 0;
}