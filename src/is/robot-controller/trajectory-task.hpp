
#pragma once

#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include <Eigen/Dense>

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

  void update(is::common::Pose const& pose);

  auto done() const -> bool override;
  auto error(is::common::Pose const& pose) const -> double override;
  auto target_pose() const -> is::common::Pose override;
  auto target_speed() const -> is::common::Speed override;
  auto rate() const -> double override;
};

TrajectoryTask::TrajectoryTask(is::robot::RobotTask const& task) {
  target = 0;

  positions = std::vector<is::common::Position>{task.trajectory().positions().begin(),
                                                task.trajectory().positions().end()};
  speeds = std::vector<is::common::Speed>{task.trajectory().speeds().begin(),
                                          task.trajectory().speeds().end()};
  if (positions.size() != speeds.size()) {
    throw std::runtime_error{"Size of positions and speeds vectors must be equal"};
  }

  allowed_error = task.allowed_error();
  frequency = task.sampling().frequency().value();
  start = std::chrono::system_clock::now();
}

auto TrajectoryTask::done() const -> bool {
  return target == positions.size();
}

void TrajectoryTask::update(is::common::Pose const& pose) {
  if (done()) { return; }

  auto D = Eigen::VectorXd{2};
  D << (positions[target].x() - pose.position().x(), positions[target].y() - pose.position().y());
  auto distance = D.norm();

  if (distance < allowed_error) { ++target; }
}

auto TrajectoryTask::error(is::common::Pose const& pose) const -> double {
  auto error = 0.0;
  if (!done()) {
    auto D = Eigen::VectorXd{2};
    D << (positions[target].x() - pose.position().x(), positions[target].y() - pose.position().y());
    error += D.norm();
  }
  return error;
}

auto TrajectoryTask::target_pose() const -> is::common::Pose {
  auto pose = is::common::Pose{};
  *pose.mutable_position() = positions.at(target);
  return pose;
}

auto TrajectoryTask::target_speed() const -> is::common::Speed {
  return speeds.at(target);
}

auto TrajectoryTask::rate() const -> double {
  return frequency;
}

}  // namespace is