
#pragma once

#include <is/msgs/camera.pb.h>
#include <is/msgs/common.pb.h>
#include <chrono>
#include <cmath>
#include <is/wire/core.hpp>
#include "extended-kalman-filter.hpp"

namespace is {

/* Implements a pose fusion and estimation algorithm. */
class PoseEstimation {
  ExtendedKalmanFilter kf;
  std::chrono::system_clock::time_point last_observation;
  std::chrono::microseconds time_since_observation;
  Eigen::VectorXd speed;
  is::Message last_message;

 public:
  PoseEstimation();

  // Returns an estimate of the pose at the current moment.
  auto pose() -> is::common::Pose;
  // Returns context of last observation received
  auto last_context(std::shared_ptr<opentracing::Tracer> const& tracer)
      -> opentracing::expected<std::unique_ptr<opentracing::SpanContext>>;
  // Configure the current robot speed. This is used to estimate the pose.
  void set_speed(is::common::Speed const&);
  auto time_since_last_observation() const -> std::chrono::microseconds;

  // Watch for FrameTransformation messages and process them to produce the current pose estimation.
  void run(is::Message const& message);

 public:
  std::function<void(std::string const&)> on_new_measurement;
};

}  // namespace is