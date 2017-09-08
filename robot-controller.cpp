#include "msgs/robot-controller.hpp"
#include "robot-parameters.hpp"

#include <algorithm>
#include <armadillo>
#include <atomic>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <is/is.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>
#include <map>
#include <mutex>
#include <thread>

using namespace arma;
using namespace std::chrono_literals;
using namespace std::chrono;
using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::geometry;
using namespace is::msg::controller;
namespace po = boost::program_options;

template <typename I>
arma::vec mean_pose(I first, I last) {
  auto join_poses = [&](auto& total, auto& msg) {
    auto pose = is::msgpack<optional<Pose>>(msg.second);
    if (pose) {
      mat current{(*pose).position.x, (*pose).position.y, std::sin((*pose).heading), std::cos((*pose).heading)};
      total = join_vert(total, current);
    }
    return total;
  };
  arma::mat poses = std::accumulate(first, last, arma::mat(0, 0, arma::fill::zeros), join_poses);

  if (!poses.empty()) {
    poses = arma::mean(poses);
    poses(0, 2) = std::atan2(poses(0, 2), poses(0, 3));
    poses = poses.cols(0, 2).t();
  }
  return arma::vectorise(poses);
}

void send_speed(is::ServiceClient& client, std::string const& robot, arma::vec const& speed) {
  Speed command{speed(0), speed(1)};
  auto id = client.request(robot + ".set_speed", is::msgpack(command));
  client.receive_for(milliseconds(1), id, is::policy::discard_others);
}

arma::vec eval_speed(robot::Parameters const& parameters, arma::vec const& current_pose, arma::vec const& desired_pose,
                     arma::vec const& trajectory_speed, double const& stop_distance) {
  vec error = desired_pose.subvec(0, 1) - current_pose.subvec(0, 1);
  if (arma::norm(error) < stop_distance)
    return arma::vec({0.0, 0.0});
  auto heading = current_pose(2);
  auto a = parameters.center_offset;
  auto L = parameters.L;
  auto K = parameters.K;
  mat invA = {{cos(heading), sin(heading)}, {-(1.0 / a) * sin(heading), (1.0 / a) * cos(heading)}};
  vec C = trajectory_speed + L % tanh((K / L) % error);
  return invA * C;
}

namespace fence {

arma::mat limit_pose(arma::vec const& current_pose, arma::vec const& desired_pose, arma::vec const& fence) {
  auto x = current_pose(0);
  auto y = current_pose(1);
  auto xd = desired_pose(0);
  auto yd = desired_pose(1);
  auto x_min = fence(0);
  auto x_max = fence(1);
  auto y_min = fence(2);
  auto y_max = fence(3);
  auto a = (yd - y) / (xd - x);
  auto b = 1 / a;
  auto x_out = xd > x_max || xd < x_min;
  auto y_out = yd > y_max || yd < y_min;
  if (x_out || y_out) {
    is::log::warn("Desired pose out of bounds. Limiting it.");
  }
  auto xd_b = x_out ? std::min(std::max(xd, x_min), x_max) : xd;
  auto yd_b = y_out ? std::min(std::max(yd, y_min), y_max) : yd;
  xd_b = !x_out && y_out ? b * (yd_b - y) + x : xd_b;
  yd_b = x_out && !y_out ? a * (xd_b - x) + y : yd_b;
  arma::mat new_desired_pose(desired_pose);
  new_desired_pose(0) = xd_b;
  new_desired_pose(1) = yd_b;
  return new_desired_pose;
}

arma::mat limit_speed(arma::vec const& current_pose, arma::vec const& speed, arma::vec const& fence) {
  auto x = current_pose(0);
  auto y = current_pose(1);
  auto x_min = fence(0);
  auto x_max = fence(1);
  auto y_min = fence(2);
  auto y_max = fence(3);
  auto x_out = x > x_max || x < x_min;
  auto y_out = y > y_max || y < y_min;
  if (x_out || y_out) {
    is::log::warn("Robot leaving critical area. Forcing stop.");
    return arma::mat(2, 1, arma::fill::zeros);
  }
  return speed;
}

}  // ::fence

namespace is {
template <typename Time>
auto consume_until(is::Connection& is, is::QueueInfo const& tag, Time const& deadline) {
  auto timeout = deadline - high_resolution_clock::now();
  Envelope::ptr_t envelope;
  if (duration_cast<milliseconds>(timeout).count() >= 1.0) {
    envelope = is.consume_for(tag, timeout);
  }
  return envelope;
}
}  // ::is

int main(int argc, char* argv[]) {
  std::string uri;
  std::string robot;
  std::vector<std::string> sources;
  std::string parameters_file;
  double rate;

  double desired_x;
  double desired_y;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://edge.is:30000"), "broker uri");
  options("robot,r", po::value<std::string>(&robot), "robot name");
  options("sources,s", po::value<std::vector<std::string>>(&sources)->multitoken(),
          "the list of topics where this service will consume poses");
  options("parameters,p", po::value<std::string>(&parameters_file)->default_value("parameters.yaml"),
          "yaml file with robot parameters");
  options("rate,R", po::value<double>(&rate)->default_value(5.0), "sampling rate");

  options("desired_x,x", po::value<double>(&desired_x)->default_value(0.0), "desired x pose");
  options("desired_y,y", po::value<double>(&desired_y)->default_value(0.0), "desired y pose");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("robot")) {
    std::cout << description << std::endl;
    return 1;
  }

  robot::Parameters parameters(parameters_file);

  Trajectory trajectory;
  std::mutex mtx;
  std::condition_variable new_task;
  bool trajectory_received = false;

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  std::string name = "robot-controller" + robot.substr(robot.find_last_of('.'));
  is::log::info("Starting service {}", name);
  // clang-format off
  auto thread = is::advertise(uri, name, {
    {
      "do_trajectory", [&](is::Request request) -> is::Reply {
        mtx.lock();
        trajectory = is::msgpack<Trajectory>(request);
        trajectory_received = true;
        mtx.unlock();
        is::log::info("New trajectory received with {} points", trajectory.positions.size());
        new_task.notify_one();
        return is::msgpack(status::ok);
      }
    }
  });
  // clang-format on

  std::vector<std::string> topics;
  std::transform(std::begin(sources), std::end(sources), std::back_inserter(topics),
                 [](auto& s) { return s + ".pose"; });

  // Waiting first task
  // std::unique_lock<std::mutex> lk(mtx);
  // new_task.wait(lk, [&] { return trajectory_received; });

  auto tag = is.subscribe(topics);

  std::map<std::string, is::Envelope::ptr_t> messages;
  vec current_pose;
  vec desired_pose({desired_x, desired_y, 0.0});
  vec trajectory_speed({0.0, 0.0});
  enum State { CONSUMING, REFRESH_DEADLINE, WAIT_NEXT_LOOP, SEND_COMMAND, STOP_ROBOT };
  State state = CONSUMING;

  auto period = static_cast<unsigned int>(1000.0 / rate);
  auto deadline = high_resolution_clock::now() + milliseconds(period);
  while (1) {
    switch (state) {
      case CONSUMING: {
        auto envelope = is::consume_until(is, tag, deadline);
        if (envelope == nullptr) {
          state = STOP_ROBOT;
          break;
        }
        is::log::info("New message from {}", envelope->RoutingKey());
        messages.emplace(std::make_pair(envelope->RoutingKey(), envelope));
        if (messages.size() == topics.size()) {
          current_pose = mean_pose(messages.begin(), messages.end());
          state = SEND_COMMAND;
        }
        break;
      }
      case REFRESH_DEADLINE: {
        messages.clear();
        deadline += milliseconds(period);
        state = CONSUMING;
        break;
      }
      case WAIT_NEXT_LOOP: {
        is::Envelope::ptr_t envelope;
        while (1) {
          envelope = is::consume_until(is, tag, deadline);
          if (envelope == nullptr)
            break;
          is::log::info("Discarding messages from {}", envelope->RoutingKey());
        }
        state = REFRESH_DEADLINE;
        break;
      }
      case SEND_COMMAND: {
        if (current_pose.empty()) {
          is::log::warn("Pattern not found");
          state = STOP_ROBOT;
          break;
        }
        desired_pose = fence::limit_pose(current_pose, desired_pose, parameters.fence);
        arma::vec speed = eval_speed(parameters, current_pose, desired_pose, trajectory_speed, 200.0);
        speed = fence::limit_speed(current_pose, speed, parameters.fence);
        send_speed(client, robot, speed);
        is::log::info("Speed command sent: {},{}", speed(0), speed(1));
        state = WAIT_NEXT_LOOP;
        break;
      }
      case STOP_ROBOT: {
        send_speed(client, robot, arma::mat(2, 1, fill::zeros));
        is::log::warn("Stopping robot");
        state = REFRESH_DEADLINE;
        break;
      }
    }
  }

  thread.join();
  return 0;
}