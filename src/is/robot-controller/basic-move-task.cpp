
#include "basic-move-task.hpp"
#include <is/wire/core/logger.hpp>

namespace is {

BasicMoveTask::BasicMoveTask(is::robot::BasicMoveTask const& task) : target(0) {
  if (task.speeds_size() != 0 and task.speeds_size() != task.positions_size()) {
    throw std::runtime_error{
        fmt::format("Speeds and positions must have the same size. User passed: size(speeds)={} "
                    "size(positions)={}",
                    task.speeds_size(), task.positions_size())};
  }

  positions = std::vector<is::common::Position>{task.positions().begin(), task.positions().end()};
  speeds = std::vector<is::common::Speed>{task.speeds().begin(), task.speeds().end()};
  allowed_error = task.allowed_error();
  frequency = task.rate();
  start = std::chrono::system_clock::now();
}

auto BasicMoveTask::done() const -> bool {
  return target == positions.size();
}

auto BasicMoveTask::completion() const -> double {
  return positions.size() != 0 ? target / static_cast<double>(positions.size()) : 1.0;
}

void BasicMoveTask::update(is::common::Pose const& pose) {
  if (!done()) {
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

auto BasicMoveTask::error(is::common::Pose const& pose) const -> double {
  auto error = 0.0;
  if (!done()) {
    auto x = positions[target].x() - pose.position().x();
    auto y = positions[target].y() - pose.position().y();
    error += std::sqrt(x * x + y * y);
  }
  return error;
}

auto BasicMoveTask::target_pose() const -> is::common::Pose {
  auto pose = is::common::Pose{};
  if (target < positions.size()) *pose.mutable_position() = positions[target];
  return pose;
}

auto BasicMoveTask::target_speed() const -> is::common::Speed {
  return target < speeds.size() ? speeds[target] : is::common::Speed{};
}

auto BasicMoveTask::rate() const -> double {
  return frequency;
}

auto BasicMoveTask::began_at() const -> std::chrono::system_clock::time_point {
  return start;
}

auto BasicMoveTask::ended_at() const -> std::chrono::system_clock::time_point {
  return end;
}

}  // namespace is