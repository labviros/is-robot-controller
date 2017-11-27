/*

{
"pose" : { "goal" : { "position" : {"x": 0.0, "y": 0.0} }}, "allowed_error": 0.1,
"sampling" : {"frequency": 5.0}
}

*/

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

#include <is/msgs/common.pb.h>
#include <is/msgs/image.pb.h>
#include <is/msgs/robot.pb.h>
#include <is/is.hpp>

#include "fence.hpp"
#include "robot-parameters.hpp"
#include "task.hpp"

using namespace arma;
using namespace is::robot;
using namespace is::common;
using namespace is::vision;

is::pb::Timestamp eval_initial_deadline(is::rmq::Channel::ptr_t& channel, std::string const& queue,
                                        std::vector<std::string> const& cameras, is::pb::Duration const& period) {
  is::info("Evaluating initial deadline");
  std::vector<std::string> topics;
  std::transform(cameras.begin(), cameras.end(), std::back_inserter(topics),
                 [](auto& camera) { return fmt::format("{}.Timestamp", camera); });

  is::subscribe(channel, queue, topics);
  std::map<std::string, is::pb::Timestamp> timestamps;
  for (;;) {
    auto envelope = is::consume(channel, queue);
    auto routing_key = envelope->RoutingKey();
    auto pos = routing_key.find_last_of('.');
    if (pos == std::string::npos || routing_key.substr(pos + 1) != "Timestamp")
      continue;

    auto timestamp = *is::unpack<is::pb::Timestamp>(envelope);
    timestamps[routing_key] = timestamp;

    const auto ts_nanos = [](auto const& ts) { return is::pb::TimeUtil::TimestampToNanoseconds(ts); };
    const auto dt_nanos = [](auto const& dt) { return is::pb::TimeUtil::DurationToNanoseconds(dt); };

    if (cameras.size() == timestamps.size()) {
      auto minmax = std::minmax_element(timestamps.begin(), timestamps.end(), [&](auto lhs, auto rhs) {
        return ts_nanos(lhs.second) < ts_nanos(rhs.second);
      });

      auto min = (*minmax.first).second;
      auto max = (*minmax.second).second;
      auto diff = ts_nanos(max) - ts_nanos(min);
      if (diff < dt_nanos(period) / 5.0) {
        is::unsubscribe(channel, queue, topics);
        return min + period;
      }
    }
  }
}

template <typename I>
arma::vec mean_pose(I first, I last, unsigned int const robot_id) {
  auto join_poses = [&](auto& total, auto& msg) {
    auto ia = *is::unpack<ImageAnnotations>(msg.second);
    auto begin = ia.annotations().begin();
    auto end = ia.annotations().end();
    auto pos = std::find_if(
        begin, end, [&](auto image_annotation) { return image_annotation.label() == std::to_string(robot_id); });

    if (pos != end) {
      if (!pos->has_pose())
        return total;
      auto x = static_cast<double>(pos->pose().position().x());
      auto y = static_cast<double>(pos->pose().position().y());
      auto th = static_cast<double>(pos->pose().orientation().roll());
      mat current{x, y, std::sin(th), std::cos(th)};
      total = join_vert(total, current);
    }
    return total;
  };

  arma::mat poses = std::accumulate(first, last, arma::mat(0, 0, arma::fill::zeros), join_poses);
  is::info("{} / {}", poses.n_rows, std::distance(first, last));

  if (!poses.empty()) {
    poses = arma::mean(poses);
    poses(0, 2) = std::atan2(poses(0, 2), poses(0, 3));
    poses = poses.cols(0, 2).t();
  }
  return arma::vectorise(poses);
}

bool inside_window(is::rmq::Envelope::ptr_t const& envelope, is::pb::Timestamp const& ts) {
  auto ts_ms = is::pb::TimeUtil::TimestampToMilliseconds(ts);
  auto msg_ts_ms = static_cast<int64_t>(envelope->Message()->Timestamp());
  return msg_ts_ms > ts_ms;
}

std::string send_speed(is::rmq::Channel::ptr_t& channel, std::string const& queue, unsigned int robot_id,
                       Speed const& speed) {
  RobotConfig robot_config;
  *robot_config.mutable_speed() = speed;
  return is::request(channel, queue, fmt::format("RobotGateway.{}.SetConfig", robot_id), robot_config);
}

std::string stop_robot(is::rmq::Channel::ptr_t& channel, std::string const& queue, unsigned int robot_id) {
  Speed speed;
  return send_speed(channel, queue, robot_id, speed);
}

double get_period(SyncRequest const& sync_request) {
  if (sync_request.sampling().rate_case() == SamplingSettings::RateCase::kFrequency) {
    return 1.0 / sync_request.sampling().frequency();
  } else {
    return sync_request.sampling().period();
  }
}

namespace std {
std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& vec) {
  for (auto item : vec) {
    os << item << " ";
  }
  return os;
}
}  // namespace std

int main(int argc, char* argv[]) {
  std::string uri;
  unsigned int robot_id;
  std::vector<std::string> cameras;
  std::vector<std::string> sources;
  std::string parameters_file;
  float rate;

  is::po::options_description opts("Options");
  auto&& opt_add = opts.add_options();

  opt_add("help,", "show available options");
  opt_add("uri,u", is::po::value<std::string>(&uri)->default_value("amqp://rmq.is:30000"), "broker uri");
  opt_add("robot-id,r", is::po::value<unsigned int>(&robot_id)->default_value(0), "robot id");
  opt_add("cameras,c", is::po::value<std::vector<std::string>>(&cameras)->multitoken()->default_value(
                           {"CameraGateway.0", "CameraGateway.1", "CameraGateway.2", "CameraGateway.3"}),
          "the list of cameras");
  opt_add("sources,s", is::po::value<std::vector<std::string>>(&sources)->multitoken()->default_value(
                           {"ArUco.0", "ArUco.1", "ArUco.2", "ArUco.3"}),
          "the list of topics where this service will consume poses");
  opt_add("parameters,p", is::po::value<std::string>(&parameters_file)->default_value("parameters.yaml"),
          "yaml file with robot parameters");
  opt_add("rate,R", is::po::value<float>(&rate)->default_value(5.0), "sampling rate");

  is::parse_program_options(argc, argv, opts);

  robot::Parameters parameters(parameters_file);
  std::function<RobotControllerProgress(arma::vec)> task = task::none(parameters);
  bool do_sync;

  SyncRequest sync_request;
  std::transform(cameras.begin(), cameras.end(), is::pb::RepeatedFieldBackInserter(sync_request.mutable_entities()),
                 [](auto& camera) { return camera; });
  sync_request.mutable_sampling()->set_frequency(rate);

  is::pb::Duration period =
      is::pb::TimeUtil::NanosecondsToDuration(static_cast<int64_t>(1e9 * get_period(sync_request)));

  is::info("Trying to connect to {}", uri);
  auto channel = is::rmq::Channel::CreateFromUri(uri);

  is::ServiceProvider provider;
  provider.connect(channel);
  auto provider_tag = provider.declare_queue(fmt::format("RobotController.{}", robot_id));

  provider.delegate<RobotTask, is::pb::Empty>(
      provider_tag, "SetTask", [&](RobotTask const& robot_task, is::pb::Empty*) -> Status {
        is::info("New task received");
        robot_task.PrintDebugString();
        switch (robot_task.Task_case()) {
          case RobotTask::kPose:
            if (!robot_task.pose().has_goal())
              return is::make_status(StatusCode::INVALID_ARGUMENT, "Robot goal field not set");
            task = task::final_position(parameters, robot_task);
            break;
          case RobotTask::kPath:
            task = task::path(parameters, robot_task);
            break;
          case RobotTask::kTrajectory:
            task = task::trajectory(parameters, robot_task);
            break;
          case RobotTask::TASK_NOT_SET:
            return is::make_status(StatusCode::INVALID_ARGUMENT, "Robot task field not set");
        }
        if (!robot_task.has_sampling())
          return is::make_status(StatusCode::INVALID_ARGUMENT, "Sampling settings not set");
        do_sync = true;
        *sync_request.mutable_sampling() = robot_task.sampling();
        return is::make_status(StatusCode::OK);
      });

  auto window_end = is::current_time();

  std::vector<std::string> topics;
  std::transform(sources.begin(), sources.end(), std::back_inserter(topics),
                 [](auto& s) { return fmt::format("{}.Detection", s); });

  auto queue = is::declare_queue(channel);
  is::subscribe(channel, queue, topics);

  std::map<std::string, is::rmq::Envelope::ptr_t> messages;
  vec current_pose;
  enum State { CONSUMING, COMPUTE_COMMAND, WAIT_WINDOW_END, SYNCING };
  State state = SYNCING;

  while (1) {
    auto sampling_deadline = window_end - 0.1 * period;
    auto window_begin = window_end - period;

    switch (state) {
      case SYNCING: {
        auto time_to_sync = is::pb::TimeUtil::SecondsToDuration(100.0 * get_period(sync_request));
        auto deadline_to_sync = time_to_sync + is::current_time();
        is::info("Sending sync request. Waiting {} for reply.", time_to_sync);

        auto id = is::request(channel, queue, "Time.Sync", sync_request);
        for (;;) {
          auto envelope = is::consume_until(channel, queue, deadline_to_sync);
          if (envelope == nullptr) {
            is::warn("The sync service didn't respond");
            state = SYNCING;
            break;
          } else if (envelope->Message()->CorrelationIdIsSet() && id == envelope->Message()->CorrelationId()) {
            auto sync_status = is::rpc_status(envelope);
            if (sync_status.code() != StatusCode::OK) {
              is::warn("{}", sync_status);
              state = SYNCING;
              break;
            }
            period = is::pb::TimeUtil::NanosecondsToDuration(static_cast<int64_t>(1e9 * get_period(sync_request)));
            window_end = eval_initial_deadline(channel, queue, cameras, period);
            is::info("Sync succesfull! Period {} | Window end {}", period, window_end);
            do_sync = false;
            state = CONSUMING;
            break;
          }
        }
        break;
      }

      case CONSUMING: {
        auto envelope = is::consume_until(channel, sampling_deadline);
        if (envelope == nullptr) {
          is::info("Sampling deadline exceeded");
          state = COMPUTE_COMMAND;
          break;
        }
        if (envelope->Message()->CorrelationIdIsSet())
          break;
        if (envelope->ConsumerTag() == queue) {
          messages[envelope->RoutingKey()] = envelope;
          auto received_n = std::count_if(messages.begin(), messages.end(), [&](auto key_value) {
            return inside_window(key_value.second, window_begin);
          });
          if (received_n == sources.size()) {
            is::info("Received all messages");
            state = COMPUTE_COMMAND;
          }
        } else {
          is::info("New task received {}", envelope->RoutingKey());
          assert(envelope->ConsumerTag() == provider.get_tag());
          provider.serve(envelope);
        }
        break;
      }

      case COMPUTE_COMMAND: {
        for (auto it = messages.begin(); it != messages.end(); ++it) {
          if (!inside_window(it->second, window_begin)) {
            messages.erase(it);
          }
        }

        current_pose = mean_pose(messages.begin(), messages.end(), robot_id);
        if (current_pose.empty()) {
          is::info("Pose: ?, ?, ?");
        } else {
          is::info("Pose: {}, {}, {}", 1000.0 * current_pose(0), 1000.0 * current_pose(1),
                   current_pose(2) * (45.0 / atan(1)));
        }
        auto controller_status = task(current_pose);
        *controller_status.mutable_current_speed() = fence::limit_speed(controller_status, parameters.fence);
        send_speed(channel, queue, robot_id, controller_status.current_speed());
        is::info("Speed command sent: {}", controller_status.current_speed());
        is::publish(channel, fmt::format("RobotController.{}.Status", robot_id), controller_status);

        state = WAIT_WINDOW_END;
        break;
      }

      case WAIT_WINDOW_END: {
        auto envelope = is::consume_until(channel, queue, window_end);
        if (envelope != nullptr) {
          messages[envelope->RoutingKey()] = envelope;
          break;
        }

        window_end += period;
        if (do_sync)
          stop_robot(channel, queue, robot_id);
        state = do_sync ? SYNCING : CONSUMING;
        break;
      }
    }
  }

  return 0;
}