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

int64_t eval_initial_deadline(is::Connection& is, std::vector<std::string> const& cameras, double rate) {
  is::log::info("Evaluating initial deadline");
  std::vector<std::string> ts_topics;
  std::transform(std::begin(cameras), std::end(cameras), std::back_inserter(ts_topics),
                 [](auto& s) { return s + ".timestamp"; });

  auto ts_tag = is.subscribe(ts_topics);
  std::map<std::string, int64_t> cameras_ts;
  while (1) {
    auto envelope = is.consume(ts_tag);
    auto timestamp = is::msgpack<Timestamp>(envelope);
    cameras_ts[envelope->RoutingKey()] = timestamp.nanoseconds;

    if (cameras_ts.size() == ts_topics.size()) {
      auto ts = std::minmax_element(cameras_ts.begin(), cameras_ts.end(),
                                    [](auto lhs, auto rhs) { return lhs.second < rhs.second; });

      auto min = (*(ts.first)).second;
      auto max = (*(ts.second)).second;
      auto diff = (max - min) / 1e6;
      if (diff < (1000.0 / rate) / 5.0) {
        is.unsubscribe(ts_tag);
        return min + static_cast<int64_t>(1e9 / rate);
      }
    }
  }
}

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

void send_speed(is::ServiceClient& client, std::string const& robot, Speed const& speed) {
  auto id = client.request(robot + ".set_speed", is::msgpack(speed));
  client.receive_for(milliseconds(1), id, is::policy::discard_others);
}

void publish_data(is::Connection& is, std::string const& name, RobotControllerStatus const& status) {
  is.publish(name + ".pose", is::msgpack(status.current_pose));
  is.publish(name + ".status", is::msgpack(status));
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

namespace std {
std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& vec) {
  for (auto item : vec) {
    os << item << " ";
  }
  return os;
}
}  // ::std

int main(int argc, char* argv[]) {
  std::string uri;
  std::string robot;
  std::vector<std::string> cameras;
  std::vector<std::string> sources;
  std::string parameters_file;
  double rate;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://edge.is:30000"), "broker uri");
  options("robot,r", po::value<std::string>(&robot)->default_value("robot.0"), "robot name");
  options("cameras,c", po::value<std::vector<std::string>>(&cameras)->multitoken()->default_value(
                           {"ptgrey.0", "ptgrey.1", "ptgrey.2", "ptgrey.3"}),
          "the list of cameras");
  options("sources,s", po::value<std::vector<std::string>>(&sources)->multitoken()->default_value(
                           {"aruco-tracker.0", "aruco-tracker.1", "aruco-tracker.2", "aruco-tracker.3"}),
          "the list of topics where this service will consume poses");
  options("parameters,p", po::value<std::string>(&parameters_file)->default_value("parameters.yaml"),
          "yaml file with robot parameters");
  options("rate,R", po::value<double>(&rate)->default_value(5.0), "sampling rate");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << description << std::endl;
    return 1;
  }

  robot::Parameters parameters(parameters_file);

  std::mutex mtx;
  std::function<RobotControllerStatus(arma::vec)> task = task::none(parameters);
  std::atomic_bool do_sync;
  SamplingRate sampling_rate_task;
  sampling_rate_task.rate = rate;

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  std::string name = "robot-controller" + robot.substr(robot.find_last_of('.'));
  auto provider = is::ServiceProvider(name, is::make_channel(uri));

  provider.expose("do-task", [&](is::Request request) -> is::Reply {
    auto robot_task = is::msgpack<RobotTask>(request);
    auto positions = robot_task.positions.size();
    auto speeds = robot_task.speeds.size();
    auto desired_heading = robot_task.desired_heading;

    std::function<RobotControllerStatus(arma::vec)> new_task;
    if (positions == 1 && speeds == 0) {
      new_task = task::final_position(parameters, robot_task);
      is::log::info("[New Task] Final position");
    } else if (positions == speeds && positions > 0 && speeds > 0) {
      new_task = task::trajectory(parameters, robot_task);
      is::log::info("[New Task] Trajectoty");
    } else if (positions > 1 && speeds == 0) {
      new_task = task::path(parameters, robot_task);
      is::log::info("[New Task] Path");
    } else if (desired_heading) {
      new_task = task::final_heading(parameters, robot_task);
      is::log::info("[New Task] Final heading");
    } else {
      is::log::warn("[New Task] Invalid task received");
      return is::msgpack(status::error("Invalid task"));
    }

    mtx.lock();
    task = new_task;
    sampling_rate_task = robot_task.sampling_rate;
    mtx.unlock();
    do_sync.store(true);
    return is::msgpack(status::ok);
  });

  auto thread = std::thread(&is::ServiceProvider::listen, provider);

  auto period_ns = static_cast<int64_t>(1e9 / rate);
  int64_t window_end = eval_initial_deadline(is, cameras, rate);

  std::vector<std::string> topics;
  std::transform(std::begin(sources), std::end(sources), std::back_inserter(topics),
                 [](auto& s) { return s + ".pose"; });

  auto tag = is.subscribe(topics);

  std::map<std::string, is::Envelope::ptr_t> messages;
  vec current_pose;
  enum State { CONSUMING, SAMPLING_DEADLINE_EXCEDEED, COMPUTE_COMMAND, WAIT_WINDOW_END, SYNCING };
  State state = SYNCING;

  while (1) {
    int64_t sampling_deadline = window_end - 0.1 * period_ns;
    int64_t window_begin = window_end - period_ns;
    switch (state) {
      case SYNCING: {
        auto rate = *(sampling_rate_task.rate);
        int64_t time_to_sync = 100 / rate;
        is::log::info("Sending sync request. Waiting {} seconds for reply.", time_to_sync);

        SyncRequest sync_request;
        sync_request.entities = cameras;
        mtx.lock();
        sync_request.sampling_rate = sampling_rate_task;
        mtx.unlock();

        auto sync_id = client.request("is.sync", is::msgpack(sync_request));
        auto sync_reply_msg = client.receive_for(seconds(time_to_sync), sync_id, is::policy::discard_others);
        if (sync_reply_msg == nullptr) {
          is::log::warn("The sync service didn't respond");
          state = SYNCING;
          break;
        }
        auto sync_reply = is::msgpack<Status>(sync_reply_msg);
        if (sync_reply.value == "error") {
          is::log::warn("Sync service replied with an error: {}", sync_reply.why);
          state = SYNCING;
          break;
        }

        period_ns = static_cast<int64_t>(1e9 / rate);
        window_end = eval_initial_deadline(is, cameras, rate);
        do_sync.store(false);

        is::log::info("Sync succesfull!");
        state = CONSUMING;
        break;
      }

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
        auto controller_status = task(current_pose);
        mtx.unlock();
        if (current_pose.empty()) {
          is::log::info("Pose: ?, ?, ?");
        } else {
          is::log::info("Pose: {}, {}, {}", current_pose(0), current_pose(1), current_pose(2) * (45.0 / atan(1)));
        }
        controller_status.speed = fence::limit_speed(current_pose, controller_status.speed, parameters.fence);
        send_speed(client, robot, controller_status.speed);
        is::log::info("Speed command sent: {},{}", controller_status.speed.linear, controller_status.speed.angular);
        publish_data(is, name, controller_status);
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
        if (do_sync.load())
          send_speed(client, robot, {0.0, 0.0});
        state = do_sync.load() ? SYNCING : CONSUMING;
        break;
      }
    }
  }
  thread.join();
  return 0;
}