#pragma once

#include <is/msgs/common.pb.h>
#include <algorithm>
#include <is/wire/core.hpp>
#include <regex>

static constexpr int world_frame = 1000;
static constexpr int robot_frame = 100;

namespace is {

class SubscriptionManager {
  is::Subscription subscription;
  std::vector<int> cameras;

 public:
  SubscriptionManager(is::Subscription const& s) : subscription(s) {
    subscription.subscribe("BrokerEvents.Consumers");
  }

  void run(is::Message const& message) {
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
};

}