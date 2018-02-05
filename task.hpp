#ifndef __TASK_HPP__
#define __TASK_HPP__

#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include "robot-parameters.pb.h"
#include <armadillo>
#include <cmath>
#include <functional>

namespace task {

using namespace is::robot;
using namespace is::common;

arma::vec position_to_vec(Position const& position) {
  auto x = static_cast<double>(position.x());
  auto y = static_cast<double>(position.y());
  return arma::vec({x, y, 0.0});
}

arma::vec pose_to_vec(Pose const& pose) {
  auto x = static_cast<double>(pose.position().x());
  auto y = static_cast<double>(pose.position().y());
  auto th = static_cast<double>(pose.orientation().roll());
  return arma::vec({x, y, th});
}

arma::vec speed_to_vec(Speed const& speed) {
  auto dx = static_cast<double>(speed.linear());
  auto dy = static_cast<double>(speed.angular());
  return arma::vec({dx, dy});
}

Pose vec_to_pose(arma::vec const& vec) {
  Pose pose;
  if (!vec.empty()) {
    pose.mutable_position()->set_x(vec(0));
    pose.mutable_position()->set_y(vec(1));
    pose.mutable_orientation()->set_roll(vec(2));
  }
  return pose;
}

Speed vec_to_speed(arma::vec const& vec) {
  Speed speed;
  if (!vec.empty()) {
    speed.set_linear(vec(0));
    speed.set_angular(vec(1));
  }
  return speed;
}

arma::vec make_control_pose(arma::vec current_pose, double a) {
  // if (current_pose.empty())
  //   return current_pose;
  // auto heading = current_pose(2);
  // current_pose(0) += a * cos(heading);
  // current_pose(1) += a * sin(heading);
  return current_pose;
}

// returns <speed, control_pose, arrived>
std::tuple<arma::vec, arma::vec, bool> eval_speed(Parameters const& parameters, arma::vec const& current_pose,
                                                  arma::vec const& desired_pose, double stop_distance,
                                                  bool stop_condition = true,
                                                  arma::vec const& trajectory_speed = arma::vec({0.0, 0.0})) {
  if (current_pose.empty())
    return std::make_tuple(arma::vec({0.0, 0.0}), current_pose, false);

  auto heading = current_pose(2);
  auto a = parameters.center_offset();
  arma::vec L({parameters.speed_limits(0), parameters.speed_limits(1)});
  arma::vec K({parameters.gains(0), parameters.gains(1)});
  arma::vec control_pose = make_control_pose(current_pose, a);

  arma::vec error = desired_pose.subvec(0, 1) - control_pose.subvec(0, 1);
  if (stop_condition && arma::norm(error) < stop_distance)
    return std::make_tuple(arma::vec({0.0, 0.0}), control_pose, true);
  arma::mat invA = {{cos(heading), sin(heading)}, {-(1.0 / a) * sin(heading), (1.0 / a) * cos(heading)}};
  arma::vec L1 = L / 2.0;
  arma::vec c1 = trajectory_speed;
  arma::vec c2 = L1 % tanh((K / L1) % error);
  arma::vec C = arma::min(c1 + c2, L);
  return std::make_tuple(invA * C, control_pose, false);
}

RobotControllerProgress make_status(arma::vec const& current_pose, arma::vec const& desired_pose,
                                    std::tuple<arma::vec, arma::vec, bool> const& eval_speed_output) {
  arma::vec speed = std::get<0>(eval_speed_output);
  arma::vec control_pose = std::get<1>(eval_speed_output);
  bool arrived = std::get<2>(eval_speed_output);

  RobotControllerProgress status;
  if (!control_pose.empty())
    *status.mutable_current_pose() = vec_to_pose(control_pose);
  if (!desired_pose.empty())
    *status.mutable_desired_pose() = vec_to_pose(desired_pose);
  if (!speed.empty())
    *status.mutable_current_speed() = vec_to_speed(speed);
  if (!control_pose.empty() && !desired_pose.empty())
    status.set_error(arma::norm(desired_pose.subvec(0, 1) - control_pose.subvec(0, 1)));
  status.set_done(arrived);
  return status;
}

auto none(Parameters const& parameters) {
  auto a = parameters.center_offset();
  return [=](arma::vec const& current_pose) {
    return make_status(current_pose, make_control_pose(current_pose, a),
                       std::make_tuple(arma::vec({0.0, 0.0}), make_control_pose(current_pose, a), true));
  };
}

auto final_position(Parameters const& parameters, RobotTask const& robot_task) {
  auto task = robot_task.pose();
  auto desired_pose = pose_to_vec(task.goal());
  auto stop_distance = robot_task.allowed_error();

  return [=](arma::vec const& current_pose) {
    auto eval_output = eval_speed(parameters, current_pose, desired_pose, stop_distance);
    return make_status(current_pose, desired_pose, eval_output);
  };
}

auto final_heading(Parameters const& parameters, RobotTask const& /*robot_task*/) {
  /*
  auto desired_heading = *(robot_task.desired_heading);
  auto stop_heading =
      robot_task.stop_heading ? *(robot_task.stop_heading) : 5.0 * (atan(1) / 45.0);  // default value is 5.0 deg

  return [=](arma::vec const& current_pose) {
    auto eval_output = eval_rotspeed(parameters, current_pose, desired_heading, stop_heading);
    auto desired_pose = std::get<1>(eval_output);
    if (!desired_pose.empty())
      desired_pose(2) = desired_heading;
    return make_status(current_pose, desired_pose, eval_output);
  };
  */
  return none(parameters);
}

auto trajectory(Parameters const& parameters, RobotTask const& robot_task) {
  auto task = robot_task.trajectory();

  return [
    positions = task.positions(), speeds = task.speeds(), i = std::size_t(0),
    stop_distance = robot_task.allowed_error(), parameters
  ](arma::vec const& current_pose) mutable {
    if (i == positions.size()) {
      arma::vec desired_pose = position_to_vec(positions[i - 1]);
      auto eval_output = eval_speed(parameters, current_pose, desired_pose, stop_distance);
      return make_status(current_pose, desired_pose, eval_output);
    }

    arma::vec desired_pose = position_to_vec(positions[i]);
    arma::vec trajectory_speed = speed_to_vec(speeds[i]);
    i++;

    auto eval_output =
        eval_speed(parameters, current_pose, desired_pose, stop_distance, i == positions.size(), trajectory_speed);
    return make_status(current_pose, desired_pose, eval_output);
  };
}

auto path(Parameters const& parameters, RobotTask const& /*robot_task*/) {
  /*
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
      auto eval_output = eval_speed(parameters, current_pose, desired_pose, stop_distance);
      return make_status(current_pose, desired_pose, eval_output);
    }
    if (current_pose.empty())
      return make_status(current_pose, point2vec::pose(positions[i]),
                         std::make_tuple(arma::vec({0.0, 0.0}), current_pose, false));

    arma::vec desired_pose = point2vec::pose(positions[i]);
    arma::vec error = desired_pose.subvec(0, 1) - current_pose.subvec(0, 1);
    if (arma::norm(error) < switch_distance)
      i++;

    auto eval_output = eval_speed(parameters, current_pose, desired_pose, stop_distance, false);
    return make_status(current_pose, desired_pose, eval_output);
  };
  */
  return none(parameters);
}

}  // ::task

#endif  // __TASK_HPP__