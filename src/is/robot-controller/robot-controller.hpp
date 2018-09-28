#pragma once

#include <Eigen/Dense>
#include "conf/options.pb.h"
#include "control-task.hpp"
#include "pose-estimation.hpp"

namespace is {

class RobotController {
  PoseEstimation* estimator;
  ControllerParameters parameters;
  ControlTask* task;
  std::chrono::system_clock::time_point next_deadline;

 public:
  RobotController(is::PoseEstimation*, is::ControllerParameters const&);

  auto compute_control_action() -> is::common::Speed;
  void set_task(ControlTask* new_task);
  auto run(is::Channel const& channel) -> std::chrono::system_clock::time_point;
};

RobotController::RobotController(is::PoseEstimation* est, is::ControllerParameters const& params)
    : estimator(est), parameters(params), next_deadline(std::chrono::system_clock::now()) {}

auto RobotController::compute_control_action() -> is::common::Speed {
  auto speed = is::common::Speed{};

  if (!task->done()) {
    auto gains = Eigen::VectorXd{2};
    auto target_speed = Eigen::VectorXd{2};
    auto speed_limits = Eigen::VectorXd{2};
    auto inverse_kinematics = Eigen::MatrixXd{2, 2};

    auto current_pose = estimator->pose();
    auto error = task->error(current_pose);
    auto heading = current_pose.orientation().roll();
    auto offset = parameters.center_offset();

    gains << parameters.gains(0), parameters.gains(1);
    speed_limits << parameters.speed_limits(0), parameters.speed_limits(1);
    target_speed << task->target_speed().linear(), task->target_speed().angular();
    inverse_kinematics << std::cos(heading), std::sin(heading), -std::sin(heading) / offset,
        std::cos(heading) / offset;

    Eigen::VectorXd error_action = gains.cwiseQuotient(speed_limits) * error;
    error_action = error_action.array().tanh();
    error_action = error_action.cwiseProduct(speed_limits);

    auto action = inverse_kinematics * ((target_speed + error_action).cwiseMin(speed_limits));
    speed.set_linear(action(0));
    speed.set_angular(action(1));
  }

  return speed;
}

void RobotController::set_task(ControlTask* new_task) {
  task = new_task;
}

auto RobotController::run(is::Channel const& channel) -> std::chrono::system_clock::time_point {
  if (std::chrono::system_clock::now() >= next_deadline) {
    next_deadline += std::chrono::microseconds(static_cast<int>(1e9 / task->rate()));

    auto action = compute_control_action();
    is::info("event=ControlLoop v={} w={}", action.linear(), action.angular());
  }
  return next_deadline;
}

}  // namespace is