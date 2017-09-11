#include "msgs/robot-controller.hpp"
#include "fence.hpp"
#include "robot-parameters.hpp"
#include "task.hpp"

#include <is/is.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>

#include <algorithm>
#include <armadillo>
#include <atomic>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
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
  is::log::info("{} / {}", poses.n_rows, std::distance(first, last));

  if (!poses.empty()) {
    poses = arma::mean(poses);
    poses(0, 2) = std::atan2(poses(0, 2), poses(0, 3));
    poses = poses.cols(0, 2).t();
  }
  return arma::vectorise(poses);
}

template <typename I>
bool inside_window(I first, I last, int64_t timestamp) {
  return std::all_of(first, last,
                     [&](auto msg) { return static_cast<int64_t>(msg.second->Message()->Timestamp()) > timestamp; });
}

void send_speed(is::ServiceClient& client, std::string const& robot, arma::vec const& speed) {
  Speed command{speed(0), speed(1)};
  auto id = client.request(robot + ".set_speed", is::msgpack(command));
  client.receive_for(milliseconds(1), id, is::policy::discard_others);
}

void publish_pose(is::Connection& is, std::string const& name, arma::vec const& current_pose) {
  optional<Pose> visual_pose;
  if (!current_pose.empty()) {
    Pose pose;
    pose.position.x = current_pose(0);
    pose.position.y = current_pose(1);
    pose.heading = current_pose(2);
    visual_pose = pose;
  } else {
    visual_pose = none;
  }
  is.publish(name + ".pose", is::msgpack(visual_pose));
}

namespace is {
auto consume_until(is::Connection& is, is::QueueInfo const& tag, int64_t deadline) {
  auto timeout = deadline - is::time_since_epoch_ns();
  Envelope::ptr_t envelope;
  if (timeout >= 1e6) {  // timeout > 1ms
    envelope = is.consume_for(tag, milliseconds(static_cast<int64_t>(timeout / 1e6)));
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

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("robot")) {
    std::cout << description << std::endl;
    return 1;
  }

  robot::Parameters parameters(parameters_file);

  std::mutex mtx;
  std::function<arma::vec(arma::vec)> task = task::none();

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  std::string name = "robot-controller" + robot.substr(robot.find_last_of('.'));
  auto provider = is::ServiceProvider(name, is::make_channel(uri));

  provider.expose("do-task", [&](is::Request request) -> is::Reply {
    auto robot_task = is::msgpack<RobotTask>(request);
    auto positions = robot_task.positions.size();
    auto speeds = robot_task.speeds.size();

    std::function<arma::vec(arma::vec)> new_task;
    if (positions == 1 && speeds == 0) {
      new_task = task::final_position(parameters, robot_task);
      is::log::info("[New Task] Final position");
    } else if (positions == speeds && positions > 0 && speeds > 0) {
      new_task = task::trajectory(parameters, robot_task);
      is::log::info("[New Task] Trajectoty");
    } else if (positions > 1 && speeds == 0) {
      new_task = task::path(parameters, robot_task);
      is::log::info("[New Task] Path");
    } else {
      is::log::warn("[New Task] Invalid task received");
      return is::msgpack(status::error("Invalid task"));
    }

    mtx.lock();
    task = new_task;
    mtx.unlock();
    return is::msgpack(status::ok);
  });

  auto thread = std::thread(&is::ServiceProvider::listen, provider);

  std::vector<std::string> topics;
  std::transform(std::begin(sources), std::end(sources), std::back_inserter(topics),
                 [](auto& s) { return s + ".pose"; });

  auto tag = is.subscribe(topics);

  std::map<std::string, is::Envelope::ptr_t> messages;
  vec current_pose;
  enum State { CONSUMING, SAMPLING_DEADLINE_EXCEDEED, COMPUTE_COMMAND, WAIT_WINDOW_END };
  State state = CONSUMING;

  auto period_ns = static_cast<int64_t>(1e9 / rate);
  int64_t window_end = is::time_since_epoch_ns() + period_ns;

  while (1) {
    int64_t sampling_deadline = window_end - 0.1 * period_ns;
    int64_t window_begin = window_end - period_ns;
    switch (state) {
      case CONSUMING: {
        auto envelope = is::consume_until(is, tag, sampling_deadline);
        if (envelope != nullptr) {
          messages[envelope->RoutingKey()] = envelope;

          if (inside_window(messages.begin(), messages.end(), window_begin)) {
            current_pose = mean_pose(messages.begin(), messages.end());
            state = COMPUTE_COMMAND;
          }
          break;
        }
        state = SAMPLING_DEADLINE_EXCEDEED;
        break;
      }

      case SAMPLING_DEADLINE_EXCEDEED: {
        std::map<std::string, is::Envelope::ptr_t> filtered_messages;
        std::copy_if(messages.begin(), messages.end(), std::inserter(filtered_messages, filtered_messages.begin()),
                     [&](auto msg) { return static_cast<int64_t>(msg.second->Message()->Timestamp()) > window_begin; });
        current_pose = mean_pose(filtered_messages.begin(), filtered_messages.end());
        state = COMPUTE_COMMAND;
        break;
      }

      case COMPUTE_COMMAND: {
        mtx.lock();
        arma::vec speed = task(current_pose);
        mtx.unlock();
        speed = fence::limit_speed(current_pose, speed, parameters.fence);
        send_speed(client, robot, speed);
        is::log::info("Speed command sent: {},{}", speed(0), speed(1));
        publish_pose(is, name, current_pose);
        state = WAIT_WINDOW_END;
        break;
      }

      case WAIT_WINDOW_END: {
        auto envelope = is::consume_until(is, tag, window_end);
        if (envelope != nullptr) {
          messages[envelope->RoutingKey()] = envelope;
          break;
        }
        window_end += period_ns;
        state = CONSUMING;
        break;
      }
    }
  }
  thread.join();
  return 0;
}