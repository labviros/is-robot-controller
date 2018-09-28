
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
  TrajectoryTask();

  void update(is::common::Pose const& pose);

  auto done() override const -> bool;
  auto error(is::common::Pose const& pose) override const -> double;
  auto target_position() override const -> is::common::Position const&;
  auto target_speed() override const -> is::common::Speed const&;
  auto rate() override const -> double;
};

TrajectoryTask::TrajectoryTask() {
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

auto TrajectoryTask::done() override const -> bool {
  return target == positions.size();
}

void TrajectoryTask::update(is::common::Pose const& pose) {
  if (done()) { return; }

  auto D = Eigen::VectorXd{2};
  D << (positions[target].x() - pose.position().x(), positions[target].y() - pose.position().y());
  auto distance = D.norm();

  if (distance < allowed_error) { ++target; }
}

auto TrajectoryTask::error(is::common::Pose const& pose) override const -> double {
  auto error = 0.0;
  if (!done()) {
    auto D = Eigen::VectorXd{2};
    D << (positions[target].x() - pose.position().x(), positions[target].y() - pose.position().y());
    error += D.norm();
  }
  return error;
}

auto TrajectoryTask::target_position() override const -> is::common::Position const& {
  return positions.at(target);
}
auto TrajectoryTask::target_speed() override const -> is::common::Speed const& {
  return speeds.at(target);
}
auto TrajectoryTask::rate() override const -> double {
  return frequency;
}

}  // namespace is