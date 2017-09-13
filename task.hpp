#ifndef __TASK_HPP__
#define __TASK_HPP__

#include <armadillo>
#include <cmath>
#include <functional>
#include <is/msgs/geometry.hpp>
#include "msgs/robot-controller.hpp"

namespace task {

using namespace is::msg::controller;
using namespace is::msg::geometry;

namespace point2vec {
arma::vec pose(Point const& point) {
  return arma::vec({point.x, point.y, 0.0});
}
arma::vec speed(Point const& point) {
  return arma::vec({point.x, point.y});
}
}  // ::task::point2vec

RobotControllerStatus make_status(arma::vec const& current_pose, arma::vec const& desired_pose, arma::vec const& speed,
                                  bool arrived) {
  RobotControllerStatus status;
  if (!current_pose.empty()) {
    Pose pose;
    pose.position.x = current_pose(0);
    pose.position.y = current_pose(1);
    pose.heading = current_pose(2);
    status.current_pose = pose;
  }
  if (!desired_pose.empty()) {
    Pose pose;
    pose.position.x = desired_pose(0);
    pose.position.y = desired_pose(1);
    pose.heading = desired_pose(2);
    status.desired_pose = pose;
  }
  status.speed.linear = speed(0);
  status.speed.angular = speed(1);
  status.arrived = arrived;
  return status;
}

std::pair<arma::vec, bool> eval_speed(robot::Parameters const& parameters, arma::vec const& current_pose,
                                      arma::vec const& desired_pose, double stop_distance, bool stop_condition = true,
                                      arma::vec const& trajectory_speed = arma::vec({0.0, 0.0})) {
  if (current_pose.empty())
    return std::make_pair(arma::vec({0.0, 0.0}), false);
  arma::vec error = desired_pose.subvec(0, 1) - current_pose.subvec(0, 1);
  if (stop_condition && arma::norm(error) < stop_distance)
    return std::make_pair(arma::vec({0.0, 0.0}), true);
  auto heading = current_pose(2);
  auto a = parameters.center_offset;
  auto L = parameters.L;
  auto K = parameters.K;
  arma::mat invA = {{cos(heading), sin(heading)}, {-(1.0 / a) * sin(heading), (1.0 / a) * cos(heading)}};
  arma::vec C = trajectory_speed + L % tanh((K / L) % error);
  return std::make_pair(invA * C, false);
}

auto none() {
  return [](arma::vec const& current_pose) {
    return make_status(current_pose, arma::vec(), arma::vec({0.0, 0.0}), true);
  };
}

auto final_position(robot::Parameters const& parameters, RobotTask const& robot_task) {
  auto desired_point = robot_task.positions.at(0);
  auto stop_distance = robot_task.stop_distance;
  arma::vec desired_pose = point2vec::pose(desired_point);

  return [=](arma::vec const& current_pose) {
    auto speed_arrived = eval_speed(parameters, current_pose, desired_pose, stop_distance);
    auto speed = speed_arrived.first;
    auto arrived = speed_arrived.second;
    return make_status(current_pose, desired_pose, speed, arrived);
  };
}

auto trajectory(robot::Parameters const& parameters, RobotTask const& robot_task) {
  auto task_positions = robot_task.positions;
  auto task_speeds = robot_task.speeds;
  auto stop_distance = robot_task.stop_distance;

  return [ positions = task_positions, speeds = task_speeds, i = std::size_t(0), stop_distance,
           parameters ](arma::vec const& current_pose) mutable {
    if (i == positions.size()) {
      arma::vec desired_pose = point2vec::pose(positions.back());
      auto speed_arrived = eval_speed(parameters, current_pose, desired_pose, stop_distance);
      auto speed = speed_arrived.first;
      auto arrived = speed_arrived.second;
      return make_status(current_pose, desired_pose, speed, arrived);
    }

    arma::vec desired_pose = point2vec::pose(positions[i]);
    arma::vec trajectory_speed = point2vec::speed(speeds[i]);
    i++;

    auto speed_arrived =
        eval_speed(parameters, current_pose, desired_pose, stop_distance, i == positions.size(), trajectory_speed);
    auto speed = speed_arrived.first;
    auto arrived = speed_arrived.second;
    return make_status(current_pose, desired_pose, speed, arrived);
  };
}

auto path(robot::Parameters const& parameters, RobotTask const& robot_task) {
  auto task_positions = robot_task.positions;
  auto stop_distance = robot_task.stop_distance;

  std::vector<double> distances;
  auto previous = task_positions.begin();
  std::transform(task_positions.begin() + 1, task_positions.end(), std::back_inserter(distances), [&](auto& position) {
    auto dx = (position.x - previous->x);
    auto dy = (position.y - previous->y);
    double distance = std::sqrt(dx * dx + dy * dy);
    previous++;
    return distance;
  });
  auto switch_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();

  return [ positions = task_positions, i = std::size_t(0), stop_distance, switch_distance,
           parameters ](arma::vec const& current_pose) mutable {
    if (i == positions.size()) {
      arma::vec desired_pose = point2vec::pose(positions.back());
      auto speed_arrived = eval_speed(parameters, current_pose, desired_pose, stop_distance);
      auto speed = speed_arrived.first;
      auto arrived = speed_arrived.second;
      return make_status(current_pose, desired_pose, speed, arrived);
    }
    if (current_pose.empty())
      return make_status(current_pose, point2vec::pose(positions[i]), arma::vec({0.0, 0.0}), false);

    arma::vec desired_pose = point2vec::pose(positions[i]);
    arma::vec error = desired_pose.subvec(0, 1) - current_pose.subvec(0, 1);
    if (arma::norm(error) < switch_distance)
      i++;

    auto speed_arrived = eval_speed(parameters, current_pose, desired_pose, stop_distance, false);
    auto speed = speed_arrived.first;
    auto arrived = speed_arrived.second;
    return make_status(current_pose, desired_pose, speed, arrived);
  };
}

}  // ::task

#endif  // __TASK_HPP__
