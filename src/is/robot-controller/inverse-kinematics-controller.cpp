
#include "inverse-kinematics-controller.hpp"
#include <Eigen/Dense>
#include <is/msgs/utils.hpp>

namespace is {

InverseKinematicsController::InverseKinematicsController(is::ControllerParameters const& params,
                                                         is::PoseEstimation* est)
    : parameters(params),
      estimator(est),
      task(nullptr),
      next_deadline(std::chrono::system_clock::now()),
      last_cid(0) {}

auto InverseKinematicsController::compute_control_action() -> is::robot::RobotControllerProgress {
  auto progress = is::robot::RobotControllerProgress{};

  auto have_task = task != nullptr && !task->done();
  auto no_disruption = estimator->time_since_last_observation() <
                       is::to_nanoseconds(parameters.allowed_measurement_disruption());
  if (have_task && no_disruption) {
    auto gains = Eigen::VectorXd{2};
    auto target_speed = Eigen::VectorXd{2};
    auto speed_limits = Eigen::VectorXd{2};
    auto inverse_kinematics = Eigen::MatrixXd{2, 2};

    auto offset = parameters.center_offset();
    auto current_pose = estimator->pose();
    auto error = task->error(current_pose);
    auto heading = current_pose.orientation().roll();

    gains << parameters.gains(0), parameters.gains(1);
    speed_limits << parameters.speed_limits(0), parameters.speed_limits(1);
    target_speed << task->target_speed().linear(), task->target_speed().angular();
    inverse_kinematics << std::cos(heading), std::sin(heading), -std::sin(heading) / offset,
        std::cos(heading) / offset;

    Eigen::VectorXd error_action = gains.cwiseQuotient(speed_limits) * error;
    error_action = error_action.array().tanh();
    error_action = error_action.cwiseProduct(speed_limits);

    auto action = inverse_kinematics * ((target_speed + error_action).cwiseMin(speed_limits));
    progress.mutable_current_speed()->set_linear(action(0));
    progress.mutable_current_speed()->set_angular(action(1));
    *progress.mutable_current_pose() = current_pose;
    *progress.mutable_desired_pose() = task->target_pose();
    progress.set_error(error);
  } else {
    progress.mutable_current_speed()->set_linear(0);
    progress.mutable_current_speed()->set_linear(0);
  }

  return progress;
}

void InverseKinematicsController::set_task(is::robot::RobotTask const& new_task) {
  if (!new_task.has_sampling() || !new_task.sampling().has_frequency() ||
      new_task.sampling().frequency().value() < 1) {
    throw std::runtime_error{"Trajectory sampling rate required but not specified or too low"};
  }

  if (new_task.has_trajectory()) {
    task = std::make_unique<TrajectoryTask>(new_task);
  } else if (new_task.has_path()) {
  } else if (new_task.has_pose()) {
  } else {
    throw std::runtime_error{"Atleast one task must be specified"};
  }
}

auto InverseKinematicsController::run(is::Channel const& channel,
                                      is::Subscription const& subscription,
                                      boost::optional<is::Message> const& message)
    -> std::chrono::system_clock::time_point {
  auto is_reply =
      message && message->topic() == subscription.name() && message->correlation_id() == last_cid;
  if (is_reply) {
    last_cid = 0;
    if (message->status().ok()) {
      estimator->set_speed(last_speed);
    } else {
      is::warn("event=Controller.SetConfigFailed");
    }
  }
  if (std::chrono::system_clock::now() >= next_deadline) {
    auto rate = task != nullptr ? task->rate() : 5;
    next_deadline += std::chrono::microseconds(static_cast<int>(1e6 / rate));

    auto progress = compute_control_action();
    is::info("event=Controller progress={}", progress);

    auto config = is::robot::RobotConfig{};
    *config.mutable_speed() = progress.current_speed();
    auto config_message = is::Message{config};
    config_message.set_reply_to(subscription);
    channel.publish(fmt::format("RobotGateway.{}.SetConfig", parameters.robot_id()),
                    config_message);

    auto progress_message = is::Message{progress};
    channel.publish(fmt::format("RobotGateway.{}.Progress", parameters.robot_id()),
                    progress_message);

    auto no_reply_received = last_cid != 0;
    if (no_reply_received) { is::warn("event=Controller.NoReply"); }
    last_cid = config_message.correlation_id();
    last_speed = progress.current_speed();
  }
  return next_deadline;
}

}  // namespace is