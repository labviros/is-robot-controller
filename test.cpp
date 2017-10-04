#include <armadillo>
#include <boost/program_options.hpp>
#include <is/is.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>
#include "msgs/robot-controller.hpp"

using namespace arma;
using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::geometry;
using namespace is::msg::controller;
namespace po = boost::program_options;

Point arma2point(arma::vec const& p) {
  Point point;
  point.x = p(0);
  point.y = p(1);
  return point;
}

int main(int argc, char* argv[]) {
  std::string uri;
  std::string topic;
  std::string type;
  double rate;
  double desired_x;
  double desired_y;
  double desired_heading;
  double tf;
  double x0;
  double y0;
  double phi;
  const double pi = 4 * std::atan(1.0);

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://edge.is:30000"), "broker uri");
  options("rate,R", po::value<double>(&rate)->default_value(4.0), "sampling rate");
  options("topic,T", po::value<std::string>(&topic)->default_value("robot-controller.0.do-task"),
          "topic to request robot task");
  options("type,t", po::value<std::string>(&type)->default_value("none"),
          "Task type [none/final_position/final_heading/trajectory/path/8/generic]");
  options("desired_x,x", po::value<double>(&desired_x)->default_value(0.0),
          "Desired x pose. Use with 'final_position' type. In '8' type means ax");
  options("desired_y,y", po::value<double>(&desired_y)->default_value(0.0),
          "Desired y pose. Use with 'final_position' type. In '8' type means ay");
  options("task_time,f", po::value<double>(&tf)->default_value(30.0), "total task time");
  options("x0,X", po::value<double>(&x0)->default_value(0.0), "In '8' type means 'x' center coordinate");
  options("y0,Y", po::value<double>(&y0)->default_value(0.0), "In '8' type means 'y' center coordinate");
  options("phase,p", po::value<double>(&phi)->default_value(pi / 3.0), "trajectory phase");

  options("desired_heading,h", po::value<double>(&desired_heading)->default_value(0.0),
          "Desired heading [deg]. Use with 'final_heading' type.");

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

  } else if (type == "final_heading") {
    robot_task.desired_heading = desired_heading * (pi / 180.0);
    robot_task.stop_heading = 5.0 * (pi / 180.0);
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
  } else if (type == "8") {
    const double ax = desired_x;  // [mm]
    const double ay = desired_y;  // [mm]
    const double w = 2 * pi / tf;
    double period = 1.0 / rate;

    auto positions = [&](double t, double phi) {
      Point p;
      p.x = ax * cos(w * t + phi) / 2.0 + x0;
      p.y = ay * sin(2 * (w * t + phi)) / 2.0 + y0;
      return p;
    };
    auto speeds = [&](double t, double phi) {
      Point p;
      p.x = -w * ax * sin(w * t + phi) / 2.0;
      p.y = w * ay * cos(2 * (w * t + phi));
      return p;
    };

    auto n_points = static_cast<int>(tf / period + 1.0);
    for (int t = 0; t < n_points; ++t) {
      robot_task.positions.push_back(positions(period * t, phi));
      robot_task.speeds.push_back(speeds(period * t, phi));
      is::log::info("point {} / {} | <x,y> {},{} | <dx,dy> {},{}", t + 1, n_points, positions(period * t, phi).x,
                    positions(period * t, phi).y, speeds(period * t, phi).x, speeds(period * t, phi).y);
    }
    robot_task.stop_distance = 200.0;
  } else if (type == "generic") {
    arma::mat positions;
    arma::mat speeds;
    is::log::info("Loading points from file");
    positions.load("points/positions.mat", arma::raw_ascii);
    speeds.load("points/speeds.mat", arma::raw_ascii);

    positions.each_col([&](arma::vec const& p) { robot_task.positions.push_back(arma2point(p)); });
    speeds.each_col([&](arma::vec const& p) { robot_task.speeds.push_back(arma2point(p)); });
    robot_task.stop_distance = 200.0;
  }

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  client.request(topic, is::msgpack(robot_task));
  is::log::info("Task '{}' requested", type);

  return 0;
}