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

arma::vec eval_speed(robot::Parameters const& parameters, arma::vec const& current_pose, arma::vec const& desired_pose,
                     double stop_distance, bool stop_condition = true,
                     arma::vec const& trajectory_speed = arma::vec({0.0, 0.0})) {
  if (current_pose.empty())
    return arma::vec({0.0, 0.0});
  arma::vec error = desired_pose.subvec(0, 1) - current_pose.subvec(0, 1);
  if (stop_condition && arma::norm(error) < stop_distance)
    return arma::vec({0.0, 0.0});
  auto heading = current_pose(2);
  auto a = parameters.center_offset;
  auto L = parameters.L;
  auto K = parameters.K;
  arma::mat invA = {{cos(heading), sin(heading)}, {-(1.0 / a) * sin(heading), (1.0 / a) * cos(heading)}};
  arma::vec C = trajectory_speed + L % tanh((K / L) % error);
  return invA * C;
}

auto none() {
  return [](arma::vec) { return arma::vec({0.0, 0.0}); };
}

auto final_position(robot::Parameters const& parameters, RobotTask const& robot_task) {
  auto desired_point = robot_task.positions.at(0);
  auto stop_distance = robot_task.stop_distance;
  arma::vec desired_pose = point2vec::pose(desired_point);

  return
      [=](arma::vec const& current_pose) { return eval_speed(parameters, current_pose, desired_pose, stop_distance); };
}

auto trajectory(robot::Parameters const& parameters, RobotTask const& robot_task) {
  auto positions = robot_task.positions;
  auto speeds = robot_task.speeds;
  auto stop_distance = robot_task.stop_distance;

  return [=](arma::vec const& current_pose) {
    static auto position = positions.begin();
    static auto speed = speeds.begin();

    if (position == positions.end()) {
      arma::vec desired_pose = point2vec::pose(positions.back());
      return eval_speed(parameters, current_pose, desired_pose, stop_distance);
    }

    arma::vec desired_pose = point2vec::pose(*position);
    arma::vec trajectory_speed = point2vec::speed(*speed);
    position++;
    speed++;

    return eval_speed(parameters, current_pose, desired_pose, stop_distance, position == positions.end(),
                      trajectory_speed);
  };
}

auto path(robot::Parameters const& parameters, RobotTask const& robot_task) {
  auto positions = robot_task.positions;
  auto stop_distance = robot_task.stop_distance;

  std::vector<double> distances;
  auto previous = positions.begin();
  std::transform(positions.begin() + 1, positions.end(), std::back_inserter(distances), [&](auto& position) {
    auto dx = (position.x - previous->x);
    auto dy = (position.y - previous->y);
    double distance = std::sqrt(dx * dx + dy * dy);
    previous++;
    return distance;
  });
  auto switch_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();

  return [=](arma::vec const& current_pose) {
    static auto position = positions.begin();

    if (position == positions.end()) {
      arma::vec desired_pose = point2vec::pose(positions.back());
      return eval_speed(parameters, current_pose, desired_pose, stop_distance);
    }
    if (current_pose.empty())
      return arma::vec({0.0, 0.0});

    arma::vec desired_pose = point2vec::pose(*position);
    arma::vec error = desired_pose.subvec(0, 1) - current_pose.subvec(0, 1);
    if (arma::norm(error) < switch_distance)
      position++;

    return eval_speed(parameters, current_pose, desired_pose, stop_distance, false);
  };
}

}  // ::task

#endif  // __TASK_HPP__
