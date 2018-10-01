#pragma once

#include <is/msgs/common.pb.h>
#include <algorithm>
#include <is/wire/core.hpp>
#include <regex>

namespace is {

class SubscriptionManager {
  is::Subscription subscription;
  std::vector<int> cameras;
  int robot_frame;
  int world_frame;

 public:
  SubscriptionManager(is::Subscription const& s, int robot_frame_id, int world_frame_id);
  void run(is::Message const& message);
};

}  // namespace is