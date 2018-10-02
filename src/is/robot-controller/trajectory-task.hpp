
#pragma once

#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include <cmath>
#include "control-task.hpp"

namespace is {

class TrajectoryTask : public ControlTask {
  int target;
  std::vector<is::common::Position> positions;
  std::vector<is::common::Speed> speeds;
  double allowed_error;
  double frequency;
  std::chrono::system_clock::time_point start;

 public:
  TrajectoryTask(is::robot::RobotTask const&);

  void update(is::common::Pose const&);
  auto done() const -> bool override;
  auto error(is::common::Pose const&) const -> double override;
  auto target_pose() const -> is::common::Pose override;
  auto target_speed() const -> is::common::Speed override;
  auto rate() const -> double override;
};

}  // namespace is