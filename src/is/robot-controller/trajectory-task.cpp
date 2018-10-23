
#include "trajectory-task.hpp"
#include <is/wire/core/logger.hpp>
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
  frequency = task.rate();
  start = std::chrono::system_clock::now();
}

auto TrajectoryTask::done() const -> bool {
  return target == positions.size();
}

auto TrajectoryTask::completion() const -> double {
  return target / static_cast<double>(positions.size());
}

void TrajectoryTask::update(is::common::Pose const& pose) {
  if (!done()) {
    is::info("event=TrajectoryTask.Next progress={}/{}", target, positions.size());
    if (target == positions.size() - 1) {
      if (error(pose) <= allowed_error) {
        ++target;
        end = std::chrono::system_clock::now();
      }
    } else {
      ++target;
    }
  }
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

auto TrajectoryTask::began_at() const -> std::chrono::system_clock::time_point {
  return start;
}

auto TrajectoryTask::ended_at() const -> std::chrono::system_clock::time_point {
  return end;
}

}  // namespace is