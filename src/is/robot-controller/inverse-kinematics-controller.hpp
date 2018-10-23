
#pragma once

#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include "conf/options.pb.h"
#include "control-task.hpp"
#include "pose-estimation.hpp"
#include "trajectory-task.hpp"

namespace is {

class InverseKinematicsController {
  ControllerParameters parameters;
  PoseEstimation* estimator;
  std::unique_ptr<ControlTask> task;
  std::chrono::system_clock::time_point next_deadline;
  uint64_t last_cid;
  is::common::Speed last_speed;
  std::vector<std::string> sources;

 public:
  InverseKinematicsController(is::ControllerParameters const&, is::PoseEstimation*);

  auto compute_control_action() -> is::robot::RobotControllerProgress;
  void set_task(is::robot::RobotTask const& task);
  auto run(is::Channel const&, is::Subscription const&, boost::optional<is::Message> const&)
      -> std::chrono::system_clock::time_point;

};

}  // namespace is