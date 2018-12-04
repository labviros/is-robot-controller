#pragma once

#include <is/msgs/common.pb.h>
#include <algorithm>
#include <is/wire/core.hpp>
#include <regex>

namespace is {

/* Watch broker events in order to check all available cameras, then subscribe to all the
 * FrameTransformations that connect the robot_frame to the world_frame using the available cameras.
 */
class SubscriptionManager {
  is::Subscription subscription;
  std::vector<int> cameras;
  int robot_frame;
  int world_frame;

 public:
  SubscriptionManager(is::Subscription const& s, int robot_frame_id, int world_frame_id);

  // Watch for BrokerEvents messages and process them.
  void run(is::Message const& message);
};

}  // namespace is