
#pragma once

#include <is/msgs/camera.pb.h>
#include <is/msgs/common.pb.h>
#include <chrono>
#include <cmath>
#include <is/wire/core.hpp>
#include "extended-kalman-filter.hpp"

namespace is {

class PoseEstimation {
  ExtendedKalmanFilter kf;
  std::chrono::system_clock::time_point last_observation;
  std::chrono::microseconds time_since_observation;
  Eigen::VectorXd speed;

 public:
  PoseEstimation();
  auto pose() -> is::common::Pose;
  void set_speed(is::common::Speed const&);
  auto time_since_last_observation() const -> std::chrono::microseconds;

  void run(is::Message const& message);
};

}  // namespace is