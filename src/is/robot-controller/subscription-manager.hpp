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

SubscriptionManager::SubscriptionManager(is::Subscription const& s, int robot_frame_id,
                                         int world_frame_id)
    : subscription(s), robot_frame(robot_frame_id), world_frame(world_frame_id) {
  subscription.subscribe("BrokerEvents.Consumers");
}

void SubscriptionManager::run(is::Message const& message) {
  if (message.topic() != "BrokerEvents.Consumers") { return; }

  auto list = message.unpack<is::common::ConsumerList>();
  auto available_cameras = std::vector<int>{};
  for (auto&& key_val : list->info()) {
    auto match = std::smatch{};
    auto regex = std::regex("CameraGateway\\.(\\d+)\\.GetConfig");
    if (std::regex_match(key_val.first, match, regex)) {
      available_cameras.push_back(std::stoi(match[1]));
    }
  }

  std::sort(available_cameras.begin(), available_cameras.end());

  auto new_cameras = std::vector<int>{};
  std::set_difference(available_cameras.begin(), available_cameras.end(), cameras.begin(),
                      cameras.end(), std::back_inserter(new_cameras));
  for (auto&& id : new_cameras) {
    auto topic = fmt::format("FrameTransformation.{}.{}.{}", robot_frame, id, world_frame);
    is::info("event=Subscription.New topic={}", topic);
    subscription.subscribe(topic);
  }

  auto del_cameras = std::vector<int>{};
  std::set_difference(cameras.begin(), cameras.end(), available_cameras.begin(),
                      available_cameras.end(), std::back_inserter(del_cameras));
  for (auto&& id : del_cameras) {
    auto topic = fmt::format("FrameTransformation.{}.{}.{}", robot_frame, id, world_frame);
    is::info("event=Subscription.Del topic={}", topic);
    subscription.unsubscribe(topic);
  }

  cameras = available_cameras;
}

}  // namespace is