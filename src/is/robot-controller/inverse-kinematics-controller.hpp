
#pragma once

#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include <zipkin/opentracing.h>
#include "basic-move-task.hpp"
#include "conf/options.pb.h"
#include "control-task.hpp"
#include "pose-estimation.hpp"

namespace is {

/* Implements and inverse kinematics controller. Uses the pose estimated from the PoseEstimation
 * class to guess where the robot is, then computes an error metric using the desired pose given by
 * the current task. Using this error and some other control parameters a velocity command is
 * periodically computed and sent to the robot. */
class InverseKinematicsController {
  Channel channel;
  Subscription subscription;
  std::shared_ptr<opentracing::Tracer> tracer;
  ControllerParameters parameters;
  PoseEstimation* estimator;
  std::unique_ptr<ControlTask> task;
  int64_t task_id;
  std::chrono::system_clock::time_point next_deadline;
  uint64_t last_cid;
  is::common::Speed last_speed;
  std::vector<std::string> sources;

 private:
  void warn_robot_communication() const;
  void publish_robot_speed(is::common::Speed const& speed,
                           std::unique_ptr<opentracing::Span> const& span);
  void publish_task_progress(is::robot::RobotControllerProgress progress);

 public:
  InverseKinematicsController(is::Channel const& channel, is::Subscription const& subscription,
                              std::shared_ptr<opentracing::Tracer> const& tracer,
                              is::ControllerParameters const&, is::PoseEstimation*);

  auto compute_control_action() -> is::robot::RobotControllerProgress;
  auto set_task(is::robot::RobotTaskRequest const& task) -> uint64_t;
  auto run(boost::optional<is::Message> const&) -> std::chrono::system_clock::time_point;
};

}  // namespace is