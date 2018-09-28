
#pragma once

#include <is/msgs/common.pb.h>

namespace is {

class ControlTask {
 public:
  ControlTask() = default;
  virtual ~ControlTask() {}

  virtual auto done() const -> bool = 0;
  virtual auto error(is::common::Pose const& pose) const -> double = 0;
  virtual auto target_position() const -> is::common::Position const& = 0;
  virtual auto target_speed() const -> is::common::Speed const& = 0;
  virtual auto rate() const -> double = 0;
};

}  // namespace is