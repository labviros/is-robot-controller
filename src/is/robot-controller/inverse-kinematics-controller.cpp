
#include "inverse-kinematics-controller.hpp"
#include <Eigen/Dense>
#include <is/msgs/utils.hpp>
#include <limits>
#include <random>

namespace is {

InverseKinematicsController::InverseKinematicsController(is::Channel const& channel,
                                                         is::Subscription const& subscription,
                                                         is::ControllerParameters const& params,
                                                         is::PoseEstimation* est)
    : channel(channel),
      subscription(subscription),
      parameters(params),
      estimator(est),
      next_deadline(std::chrono::system_clock::now()),
      last_cid(0),
      task_id(0) {
  estimator->on_new_measurement = [&](auto const& topic) { sources.push_back(topic); };

  auto initial_task = is::robot::BasicMoveTask{};
  initial_task.set_rate(5.0);
  task = std::make_unique<BasicMoveTask>(initial_task);
}

auto InverseKinematicsController::compute_control_action() -> is::robot::RobotControllerProgress {
  auto progress = is::robot::RobotControllerProgress{};

  auto no_disruption = estimator->time_since_last_observation() <
                       is::to_nanoseconds(parameters.allowed_measurement_disruption());

  auto current_pose = estimator->pose();
  auto desired_pose = task->target_pose();

  if (!task->done() && no_disruption) {
    auto gains = Eigen::VectorXd{2};
    auto error = Eigen::VectorXd{2};
    auto target_speed = Eigen::VectorXd{2};
    auto speed_limits = Eigen::VectorXd{2};
    auto action_limits = Eigen::VectorXd{2};
    auto inverse_kinematics = Eigen::MatrixXd{2, 2};

    auto offset = parameters.center_offset();
    auto heading = current_pose.orientation().roll();

    error << desired_pose.position().x() - current_pose.position().x(),
        desired_pose.position().y() - current_pose.position().y();
    gains << parameters.gains(0), parameters.gains(1);
    speed_limits << parameters.speed_limits(0), parameters.speed_limits(1);
    target_speed << task->target_speed().linear(), task->target_speed().angular();
    inverse_kinematics << std::cos(heading), std::sin(heading), -std::sin(heading) / offset,
        std::cos(heading) / offset;

    Eigen::VectorXd error_action = gains.cwiseQuotient(speed_limits).cwiseProduct(error);
    error_action = error_action.array().tanh();
    error_action = error_action.cwiseProduct(speed_limits);

    auto action = inverse_kinematics * (target_speed + error_action);
    action_limits << parameters.speed_limits(2), 3.14 * parameters.speed_limits(3) / 180;

    Eigen::VectorXd limited_action = action.cwiseQuotient(action_limits).array().tanh();
    limited_action = limited_action.cwiseProduct(action_limits);

    progress.mutable_current_speed()->set_linear(limited_action(0));
    progress.mutable_current_speed()->set_angular(limited_action(1));
  } else {
    progress.mutable_current_speed()->set_linear(0);
    progress.mutable_current_speed()->set_linear(0);
  }

  *progress.mutable_current_pose() = current_pose;
  *progress.mutable_desired_pose() = desired_pose;
  progress.set_error(task->error(current_pose));

  task->update(current_pose);
  return progress;
}

auto InverseKinematicsController::set_task(is::robot::RobotTaskRequest const& new_task)
    -> uint64_t {
  if (new_task.has_basic_move_task()) {
    task = std::make_unique<BasicMoveTask>(new_task.basic_move_task());
  } else {
    throw std::runtime_error{"Atleast one task must be specified"};
  }

  task_id = new_task.id() != 0 ? new_task.id() : task_id + 1;
  return task_id;
}

auto InverseKinematicsController::run(boost::optional<is::Message> const& message)
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
    next_deadline += std::chrono::microseconds(static_cast<int>(1e6 / task->rate()));
    warn_robot_communication();

    auto progress = compute_control_action();
    publish_robot_speed(progress.current_speed());
    publish_task_progress(progress);
  }
  return next_deadline;
}

void InverseKinematicsController::warn_robot_communication() const {
  auto no_reply_received = last_cid != 0;
  if (no_reply_received) { is::warn("event=Controller.NoReply"); }
}

void InverseKinematicsController::publish_task_progress(
    is::robot::RobotControllerProgress progress) {
  progress.set_id(task_id);
  *(progress.mutable_begin()) = to_timestamp(task->began_at());
  if (task->done()) { *(progress.mutable_end()) = to_timestamp(task->ended_at()); }
  progress.set_completion(task->completion());

  for (auto const& s : sources) { *(progress.mutable_sources()->Add()) = s; }
  sources.clear();

  channel.publish(fmt::format("RobotController.{}.Progress", parameters.robot_id()),
                  is::Message{progress});

  is::info("event=Controller.Progress id={} %={} v={}m/s w={}°/s x={}m y={}m heading={}°", progress.id(),
           progress.completion() * 100, progress.current_speed().linear(),
           180 * progress.current_speed().angular() / 3.14, progress.current_pose().position().x(),
           progress.current_pose().position().y(),
           180 * progress.current_pose().orientation().roll() / 3.14);
}

void InverseKinematicsController::publish_robot_speed(is::common::Speed const& speed) {
  auto config = is::robot::RobotConfig{};
  *config.mutable_speed() = speed;
  auto config_message = is::Message{config};
  config_message.set_reply_to(subscription);
  channel.publish(fmt::format("RobotGateway.{}.SetConfig", parameters.robot_id()), config_message);

  last_speed = speed;
  last_cid = config_message.correlation_id();
}

}  // namespace is