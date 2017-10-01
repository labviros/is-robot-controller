#include "msgs/robot-controller.hpp"

#include <signal.h>
#include <armadillo>
#include <boost/program_options.hpp>
#include <is/is.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>

using namespace arma;
using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::geometry;
using namespace is::msg::controller;
namespace po = boost::program_options;

std::atomic_bool receiving{true};

void sig_function(int) {
  receiving.store(false);
  is::log::info("SIGINT received");
}

arma::mat pose2arma(optional<Pose> const& pose) {
  if (pose) {
    return arma::mat({1.0, (*pose).position.x, (*pose).position.y, (*pose).heading});
  } else {
    return arma::mat(1, 4, fill::zeros);
  }
}

int main(int argc, char* argv[]) {
  std::string uri;
  std::string robot;
  std::string output;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://edge.is:30000"), "broker uri");
  options("robot,r", po::value<std::string>(&robot)->default_value("robot.0"), "robot name");
  options("output,o", po::value<std::string>(&output)->default_value("log_robot.mat"), "robot name");

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

  signal(SIGINT, sig_function); 

  arma::mat data;
  while (receiving.load()) {
    auto envelope = is.consume(tag);
    is::log::info("NEW STATUS");
    auto status = is::msgpack<RobotControllerStatus>(envelope);

    arma::mat current_data({status.speed.linear, status.speed.angular});
    current_data = join_horiz(current_data, pose2arma(status.current_pose)); 
    current_data = join_horiz(current_data, pose2arma(status.control_pose)); 
    current_data = join_horiz(current_data, pose2arma(status.desired_pose));
    current_data = join_horiz(current_data, mat({status.arrived ? 1.0 : 0.0}));
    data = join_vert(data, current_data);

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
    if (status.error) {
      is::log::info("Error {}", *status.error);
    }
    is::log::info("Speed: {},{}", status.speed.linear, status.speed.angular);
    is::log::info("Arrived: {}", status.arrived);
    
    current_data.print("current_data");
  }

  is::log::info("Saving data");
  data.save(output, raw_ascii);
  return 0;
}