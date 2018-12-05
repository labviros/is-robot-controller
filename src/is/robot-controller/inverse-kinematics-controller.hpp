
#pragma once

#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include "conf/options.pb.h"
#include "control-task.hpp"
#include "pose-estimation.hpp"
#include "basic-move-task.hpp"

namespace is {

/* Implements and inverse kinematics controller. Uses the pose estimated from the PoseEstimation
 * class to guess where the robot is, then computes an error metric using the desired pose given by
 * the current task. Using this error and some other control parameters a velocity command is
 * periodically computed and sent to the robot. */
class InverseKinematicsController {
  ControllerParameters parameters;
  PoseEstimation* estimator;
  std::unique_ptr<ControlTask> task;
  int64_t task_id;
  std::chrono::system_clock::time_point next_deadline;
  uint64_t last_cid;
  is::common::Speed last_speed;
  std::vector<std::string> sources;

 public:
  InverseKinematicsController(is::ControllerParameters const&, is::PoseEstimation*);

  auto compute_control_action() -> is::robot::RobotControllerProgress;
  auto set_task(is::robot::RobotTaskRequest const& task) -> uint64_t;
  auto run(is::Channel const&, is::Subscription const&, boost::optional<is::Message> const&)
      -> std::chrono::system_clock::time_point;
};

}  // namespace is