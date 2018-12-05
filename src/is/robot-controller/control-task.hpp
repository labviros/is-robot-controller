
#pragma once

#include <is/msgs/common.pb.h>
#include <chrono>

namespace is {

/* Interface for control tasks. */
class ControlTask {
 public:
  virtual ~ControlTask() = default;

  virtual auto rate() const -> double = 0;

  virtual auto done() const -> bool = 0;
  virtual auto completion() const -> double = 0;

  virtual void update(is::common::Pose const&) = 0;
  virtual auto error(is::common::Pose const&) const -> double = 0;

  virtual auto target_pose() const -> is::common::Pose = 0;
  virtual auto target_speed() const -> is::common::Speed = 0;

  virtual auto began_at() const -> std::chrono::system_clock::time_point = 0;
  virtual auto ended_at() const -> std::chrono::system_clock::time_point = 0;
};

}  // namespace is