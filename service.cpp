#include <algorithm>
#include <armadillo>
#include <boost/program_options.hpp>
#include <cmath>
#include <iostream>
#include <map>

#include <google/protobuf/empty.pb.h>
#include <is/msgs/common.pb.h>
#include <is/msgs/image.pb.h>
#include <is/msgs/robot.pb.h>
#include <is/is.hpp>
#include "robot-parameters.pb.h"

#include "fence.hpp"
#include "task.hpp"

using namespace arma;
using namespace is::robot;
using namespace is::common;
using namespace is::vision;

bool is_sync(std::vector<is::pb::Timestamp> const& timestamps, is::pb::Duration const& period) {
  if (timestamps.empty())
    return false;
  auto minmax = std::minmax_element(timestamps.begin(), timestamps.end());
  auto diff = *minmax.second - *minmax.first;
  auto reply = diff < 0.1 * period;
  is::info("[Sync][error = {} ms]", is::pb::TimeUtil::DurationToMilliseconds(diff));
  return reply;
}

template <typename I>
arma::vec aggregate_pose(I first, I last, unsigned int const robot_id) {
  auto join_poses = [&](auto& total, auto& msg) {
    auto ia = *is::unpack<ImageAnnotations>(msg);
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
      auto sc = static_cast<double>(pos->score());
      mat current{x, y, std::sin(th), std::cos(th), 1.0};
      total = join_vert(total, sc * sc * current);
      // mat current{x, y, std::sin(th), std::cos(th)};
      // total = join_vert(total, current);
    }
    return total;
  };

  arma::mat poses = std::accumulate(first, last, arma::mat(0, 0, arma::fill::zeros), join_poses);

  auto n_poses = poses.n_rows;
  auto n_received = std::distance(first, last);
  if (!poses.empty()) {
    poses = arma::sum(poses);
    poses = poses / poses(0, 4);
    // poses = arma::mean(poses);
    poses(0, 2) = std::atan2(poses(0, 2), poses(0, 3));
    poses = poses.cols(0, 2).t();
    arma::vec pose = arma::vectorise(poses);
    is::info("[Pose][{}/{}][{:.1f}, {:.1f}, {:.1f}]", n_poses, n_received, 1000.0 * pose(0), 1000.0 * pose(1),
             pose(2) * (45.0 / atan(1)));
    return pose;
  } else {
    is::info("[Pose][{}/{}]", n_poses, n_received);
    return arma::vec(0, arma::fill::zeros);
  }
}

bool inside_window(is::rmq::Envelope::ptr_t const& envelope, is::pb::Timestamp const& reference) {
  auto ts = is::pb::TimeUtil::MillisecondsToTimestamp(static_cast<int64_t>(envelope->Message()->Timestamp()));
  return ts >= reference;
}

double get_period(SyncRequest const& sync_request) {
  if (sync_request.sampling().rate_case() == SamplingSettings::RateCase::kFrequency) {
    return 1.0 / sync_request.sampling().frequency();
  } else {
    return sync_request.sampling().period();
  }
}

namespace is {
bool any_of(std::vector<std::string> const& container, std::string const& target) {
  return std::any_of(container.begin(), container.end(), [&](auto& c) { return c == target; });
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
  std::string uri, zipkin_host;
  uint32_t zipkin_port;
  unsigned int robot_id;
  std::vector<std::string> cameras;
  std::vector<std::string> sources;
  std::string parameters_file;
  float rate;

  is::po::options_description opts("Options");
  auto&& opt_add = opts.add_options();

  opt_add("help,", "show available options");
  opt_add("uri,u", is::po::value<std::string>(&uri)->default_value("amqp://rmq.is:30000"), "broker uri");
  opt_add("zipkin_host,z", is::po::value<std::string>(&zipkin_host)->default_value("zipkin.default"),
          "zipkin hostname");
  opt_add("zipkin_port,P", is::po::value<uint32_t>(&zipkin_port)->default_value(9411), "zipkin port");
  opt_add("robot-id,i", is::po::value<unsigned int>(&robot_id)->default_value(0), "robot id");
  opt_add("cameras,c", is::po::value<std::vector<std::string>>(&cameras)->multitoken()->default_value(
                           {"CameraGateway.0", "CameraGateway.1", "CameraGateway.2", "CameraGateway.3"}),
          "the list of cameras");
  opt_add("sources,s", is::po::value<std::vector<std::string>>(&sources)->multitoken()->default_value(
                           {"ArUco.0", "ArUco.1", "ArUco.2", "ArUco.3"}),
          "the list of topics where this service will consume poses");
  opt_add("parameters,p", is::po::value<std::string>(&parameters_file)->default_value("parameters.json"),
          "json file with robot parameters");
  opt_add("rate,r", is::po::value<float>(&rate)->default_value(5.0), "sampling rate");

  is::parse_program_options(argc, argv, opts);

  // try to load default robot parameters
  auto maybe_parameters = is::load_from_json<Parameters>(parameters_file);
  if (!maybe_parameters)
    is::critical("Can't load robot parametres from file {}. Exiting", parameters_file);
  auto parameters = *maybe_parameters;
  is::info("Parameteres loaded: {}", parameters);
  if (parameters.speed_limits_size() != 3)
    is::critical("'speed_limits' parameters field must have 3 elements, contains {}", parameters.speed_limits_size());
  if (parameters.gains_size() != 2)
    is::critical("'gains_size' parameters field must have 2 elements, contains {}.", parameters.gains_size());

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

  is::Tracer tracer(fmt::format("RobotController.{}", robot_id), zipkin_host, zipkin_port);
  std::map<std::string /* id */, std::unique_ptr<is::ot::Span>> spans;

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

    provider.delegate<Parameters, is::pb::Empty>(
      provider_tag, "SetParameters", [&](Parameters const& p, is::pb::Empty*) -> Status {
        is::info("New parameters received: {}", p);
        auto has_error = p.speed_limits_size() != 3 || p.gains_size() != 2;
        std::string error_msg;
        if (parameters.speed_limits_size() != 3)
          error_msg = fmt::format("'speed_limits' parameters field must have 3 elements, contains {}", p.speed_limits_size());
        if (parameters.gains_size() != 2)
          error_msg = fmt::format("'gains_size' parameters field must have 2 elements, contains {}", p.gains_size());
        if (has_error) {
          is::warn("{}", error_msg);
          return is::make_status(StatusCode::INVALID_ARGUMENT, error_msg);
        }
        parameters = p;
        return is::make_status(StatusCode::OK);
    });

    std::transform(sources.begin(), sources.end(), sources.begin(),
                   [](auto& s) { return fmt::format("{}.Detection", s); });
    std::transform(cameras.begin(), cameras.end(), cameras.begin(),
                   [](auto& s) { return fmt::format("{}.Timestamp", s); });

    auto queue = is::declare_queue(channel);
    is::subscribe(channel, queue, sources);
    is::subscribe(channel, queue, cameras);

    std::vector<is::rmq::Envelope::ptr_t> annotations;
    std::vector<is::pb::Timestamp> timestamps;
    std::map<std::string /* id */, std::string /* routing-key*/> requested_ids;
    is::rmq::Envelope::ptr_t envelope;

    auto request = [&](std::string const& endpoint, is::pb::Message const& message) {
      requested_ids[is::request(channel, queue, endpoint, message)] = endpoint;
    };

    vec current_pose;
    enum State { CONSUMING, COMPUTE_COMMAND, ANNOTATION_RECEIVED, TIMESTAMP_RECEIVED, REPLY_RECEIVED };
    State state = CONSUMING;
    bool waiting_sync = false;

    auto window_begin = is::current_time();
    while (1) {
      auto window_end = window_begin + period;
      auto sampling_deadline = window_end - 0.1 * period;

      switch (state) {
        case CONSUMING: {
          envelope = is::consume_until(channel, sampling_deadline);
          if (envelope == nullptr) {
            is::warn("Sampling deadline exceeded");
            state = COMPUTE_COMMAND;
          } else if (envelope->ConsumerTag() == queue) {
            if (is::any_of(sources, envelope->RoutingKey()))
              state = ANNOTATION_RECEIVED;
            else if (is::any_of(cameras, envelope->RoutingKey()))
              state = TIMESTAMP_RECEIVED;
            else
              state = REPLY_RECEIVED;
          } else {
            assert(envelope->ConsumerTag() == provider.get_tag());
            provider.serve(envelope);
          }
          break;
        }

        case ANNOTATION_RECEIVED: {
          annotations.push_back(envelope);
          auto received_n = std::count_if(annotations.begin(), annotations.end(),
                                          [&](auto e) { return inside_window(e, window_begin); });
          state = received_n == sources.size() ? COMPUTE_COMMAND : CONSUMING;
          break;
        }

        case TIMESTAMP_RECEIVED: {
          auto maybe_timestamp = is::unpack<is::pb::Timestamp>(envelope);
          if (maybe_timestamp) {
            timestamps.push_back(*maybe_timestamp);
          }
          state = CONSUMING;
          break;
        }

        case REPLY_RECEIVED: {
          auto id = envelope->Message()->CorrelationId();
          auto pos = requested_ids.find(id);
          assert(pos != requested_ids.end());
          auto status = is::rpc_status(envelope);
          if (status.code() == StatusCode::OK)
            is::info("[{}] {}", pos->second, status);
          else
            is::warn("[{}] {}", pos->second, status);
          if (pos->second == "Time.Sync")
            waiting_sync = false;
          if (pos->second == fmt::format("RobotGateway.{}.SetConfig", robot_id)) {
            auto span = spans.find(id);
            if (span != spans.end()) {
              span->second->Finish();
              spans.erase(span);
            }
          }
          requested_ids.erase(pos);
          state = CONSUMING;
          break;
        }

        case COMPUTE_COMMAND: {
          annotations.erase(std::remove_if(annotations.begin(), annotations.end(),
                                           [&](auto e) { return !inside_window(e, window_begin); }),
                            annotations.end());

          current_pose = aggregate_pose(annotations.begin(), annotations.end(), robot_id);
          auto controller_status = task(current_pose);
          *controller_status.mutable_current_speed() = fence::limit_speed(controller_status, parameters.fence());

          RobotConfig robot_config;
          auto speed = controller_status.current_speed();
          *robot_config.mutable_speed() = speed;
          auto speed_msg = is::prepare_request(queue, robot_config);
          auto correlation_id = speed_msg->CorrelationId();

          auto last_annotation = std::max_element(annotations.begin(), annotations.end(), [](auto lhs, auto rhs) {
            return lhs->Message()->Timestamp() < rhs->Message()->Timestamp();
          });
          if (last_annotation != annotations.end()) {
            auto span = tracer.extract(*last_annotation, "control");
            tracer.inject(speed_msg, span->context());
            spans[correlation_id] = std::move(span);
          }

          auto speed_endpoint = fmt::format("RobotGateway.{}.SetConfig", robot_id);
          requested_ids[speed_msg->CorrelationId()] = speed_endpoint;
          is::publish(channel, speed_endpoint, speed_msg);
          is::info("[Speed][{:.1f}, {:.1f}]", 1000.0 * speed.linear(), (45.0 / atan(1)) * speed.angular());

          is::publish(channel, fmt::format("RobotController.{}.Status", robot_id), controller_status);

          if (!is_sync(timestamps, period) && !waiting_sync) {
            is::info("[Sync] Requesting");
            request("Time.Sync", sync_request);
            waiting_sync = true;
          }

          auto min_ts = std::min_element(timestamps.begin(), timestamps.end());
          window_begin = min_ts != timestamps.end() ? *min_ts + period : window_begin + period;

          annotations.clear();
          timestamps.clear();
          state = CONSUMING;
          break;
        }
      }
    }

    return 0;
}