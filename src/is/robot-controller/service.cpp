
#include <google/protobuf/empty.pb.h>
#include <is/msgs/camera.pb.h>
#include <is/msgs/utils.hpp>
#include <is/wire/core.hpp>
#include <is/wire/rpc.hpp>
#include <is/wire/rpc/log-interceptor.hpp>
#include "conf/options.pb.h"
#include "pose-estimation.hpp"
#include "robot-controller.hpp"
#include "subscription-manager.hpp"

auto load_configuration(int argc, char** argv) -> is::RobotControllerOptions {
  auto filename = (argc == 2) ? argv[1] : "options.json";
  auto options = is::RobotControllerOptions{};
  is::load(filename, &options);
  is::validate_message(options);
  return options;
}

int main(int argc, char** argv) {
  auto options = load_configuration(argc, argv);
  is::info("event=ConfigLoaded options={}", options);

  auto channel = is::Channel{options.broker_uri()};
  auto subscription = is::Subscription{channel};
  auto service = is::ServiceProvider{channel};
  service.add_interceptor(is::LogInterceptor{});

  auto subscription_manager =
      is::SubscriptionManager{subscription, options.robot_frame_id(), options.world_frame_id()};
  auto estimator = is::PoseEstimation{};
  auto controller = is::RobotController{options.robot_id(), &estimator, options.parameters()};

  service.delegate<is::robot::RobotTask, google::protobuf::Empty>(
      "RobotController.{}.SetTask", [&](auto* ctx, auto const& request, auto* reply) {
        controller.set_task(request);
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
    next_control_deadline = controller.run(channel);
  }
}
