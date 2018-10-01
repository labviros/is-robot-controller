
#include "trajectory-task.hpp"

namespace is {

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

  auto x = positions[target].x() - pose.position().x();
  auto y = positions[target].y() - pose.position().y();
  auto distance = std::sqrt(x * x + y * y);

  if (distance < allowed_error) { ++target; }
}

auto TrajectoryTask::error(is::common::Pose const& pose) const -> double {
  auto error = 0.0;
  if (!done()) {
    auto x = positions[target].x() - pose.position().x();
    auto y = positions[target].y() - pose.position().y();
    error += std::sqrt(x * x + y * y);
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