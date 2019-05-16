#include <google/protobuf/empty.pb.h>
#include <is/msgs/camera.pb.h>
#include <is/msgs/utils.hpp>
#include <is/wire/core.hpp>
#include <is/wire/rpc.hpp>
#include <is/wire/rpc/log-interceptor.hpp>
#include <regex>
#include "conf/options.pb.h"
#include "inverse-kinematics-controller.hpp"
#include "pose-estimation.hpp"
#include "subscription-manager.hpp"

auto load_configuration(int argc, char** argv) -> is::RobotControllerOptions {
  auto filename = (argc == 2) ? argv[1] : "options.json";
  auto options = is::RobotControllerOptions{};
  is::load(filename, &options);
  is::validate_message(options);
  return options;
}

auto create_tracer(std::string const& name, std::string const& uri)
    -> std::shared_ptr<opentracing::Tracer> {
  std::smatch match;
  auto ok = std::regex_match(uri, match, std::regex("http:\\/\\/([a-zA-Z0-9\\.]+)(:(\\d+))?"));
  if (!ok) is::critical("Invalid zipkin uri \"{}\", expected http://<hostname>:<port>", uri);
  auto tracer_options = zipkin::ZipkinOtTracerOptions{};
  tracer_options.service_name = name;
  tracer_options.collector_host = match[1];
  tracer_options.collector_port = match[3].length() ? std::stoi(match[3]) : 9411;
  return zipkin::makeZipkinOtTracer(tracer_options);
}

int main(int argc, char** argv) {
  auto options = load_configuration(argc, argv);
  is::info("event=ConfigLoaded options={}", options);

  auto channel = is::Channel{options.broker_uri()};
  auto tracer = create_tracer(fmt::format("RobotController.{}", options.parameters().robot_id()),
                              options.zipkin_uri());
  channel.set_tracer(tracer);
  auto subscription = is::Subscription{channel};
  auto service = is::ServiceProvider{channel};
  service.add_interceptor(is::LogInterceptor{});

  auto subscription_manager =
      is::SubscriptionManager{subscription, options.robot_frame_id(), options.world_frame_id()};
  auto estimator = is::PoseEstimation{};
  auto controller =
      is::InverseKinematicsController{channel, subscription, tracer, options.parameters(), &estimator};

  service.delegate<is::robot::RobotTaskRequest, is::robot::RobotTaskReply>(
      fmt::format("RobotController.{}.SetTask", options.parameters().robot_id()),
      [&](auto* ctx, auto const& request, auto* reply) {
        reply->set_id(controller.set_task(request));
        return is::make_status();
      });

  auto next_control_deadline = std::chrono::system_clock::now();
  for (;;) {
    auto maybe_message = channel.consume_until(next_control_deadline);
    if (maybe_message) {
      subscription_manager.run(*maybe_message);
      estimator.run(*maybe_message);
      service.serve(*maybe_message);
    }
    next_control_deadline = controller.run(maybe_message);
  }
}
