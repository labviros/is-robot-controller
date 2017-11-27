#ifndef __TASK_HPP__
#define __TASK_HPP__

#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include <yaml-cpp/yaml.h>
#include <armadillo>
#include <cmath>
#include <functional>
#include "robot-parameters.hpp"

namespace task {

using namespace is::robot;
using namespace is::common;

arma::vec pose_to_vec(Pose const& pose) {
  auto x = static_cast<double>(pose.position().x());
  auto y = static_cast<double>(pose.position().y());
  auto th = static_cast<double>(pose.orientation().roll());
  return arma::vec({x, y, th});
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
  if (current_pose.empty())
    return current_pose;
  auto heading = current_pose(2);
  current_pose(0) += a * cos(heading);
  current_pose(1) += a * sin(heading);
  return current_pose;
}

// returns <speed, control_pose, arrived>
std::tuple<arma::vec, arma::vec, bool> eval_speed(robot::Parameters const& parameters, arma::vec const& current_pose,
                                                  arma::vec const& desired_pose, double stop_distance,
                                                  bool stop_condition = true,
                                                  arma::vec const& trajectory_speed = arma::vec({0.0, 0.0})) {
  if (current_pose.empty())
    return std::make_tuple(arma::vec({0.0, 0.0}), current_pose, false);

  auto heading = current_pose(2);
  auto a = parameters.center_offset;
  auto L = parameters.L;
  auto K = parameters.K;
  arma::vec control_pose = make_control_pose(current_pose, a);

  arma::vec error = desired_pose.subvec(0, 1) - control_pose.subvec(0, 1);
  if (stop_condition && arma::norm(error) < stop_distance)
    return std::make_tuple(arma::vec({0.0, 0.0}), control_pose, true);
  arma::mat invA = {{cos(heading), sin(heading)}, {-(1.0 / a) * sin(heading), (1.0 / a) * cos(heading)}};
  arma::vec C = trajectory_speed + L % tanh((K / L) % error);
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

/*
namespace point2vec {
arma::vec pose(Point const& point) {
  return arma::vec({point.x, point.y, 0.0});
}
arma::vec speed(Point const& point) {
  return arma::vec({point.x, point.y});
}
}  // ::task::point2vec

Point vec2point(arma::rowvec const& v) {
  Point p;
  p.x = v(0);
  p.y = v(1);
  return p;
}

RobotTask from_file(std::string const& filename) {
  YAML::Node node = YAML::LoadFile(filename);
  RobotTask robot_task;

  SamplingRate sampling_rate;
  sampling_rate.rate = node["rate"].as<double>();
  robot_task.sampling_rate = sampling_rate;
  robot_task.stop_distance = node["stop_distance"].as<double>();

  arma::mat positions = join_horiz(node["X"].as<arma::vec>(), node["Y"].as<arma::vec>());
  arma::mat speeds = join_horiz(node["dX"].as<arma::vec>(), node["dY"].as<arma::vec>());
  positions.each_row([&](arma::rowvec const& position) { robot_task.positions.push_back(vec2point(position)); });
  speeds.each_row([&](arma::rowvec const& speed) { robot_task.speeds.push_back(vec2point(speed)); });

  return robot_task;
}

RobotControllerProgress make_status(arma::vec const& current_pose, arma::vec const& desired_pose,
                                  std::tuple<arma::vec, arma::vec, bool> const& eval_speed_output) {
  arma::vec speed = std::get<0>(eval_speed_output);
  arma::vec control_pose = std::get<1>(eval_speed_output);
  bool arrived = std::get<2>(eval_speed_output);

  RobotControllerProgress status;
  if (!current_pose.empty()) {
    Pose pose;
    pose.position.x = current_pose(0);
    pose.position.y = current_pose(1);
    pose.heading = current_pose(2);
    status.current_pose = pose;
  }
  if (!control_pose.empty()) {
    Pose pose;
    pose.position.x = control_pose(0);
    pose.position.y = control_pose(1);
    pose.heading = control_pose(2);
    status.control_pose = pose;
  }
  if (!desired_pose.empty()) {
    Pose pose;
    pose.position.x = desired_pose(0);
    pose.position.y = desired_pose(1);
    pose.heading = desired_pose(2);
    status.desired_pose = pose;
  }
  if (!control_pose.empty() && !desired_pose.empty()) {
    status.error = arma::norm(desired_pose.subvec(0, 1) - control_pose.subvec(0, 1));
  }
  status.speed.linear = speed(0);
  status.speed.angular = speed(1);
  status.arrived = arrived;
  return status;
}

arma::vec make_control_pose(arma::vec current_pose, double a) {
  if (current_pose.empty())
    return current_pose;
  auto heading = current_pose(2);
  current_pose(0) += a * cos(heading);
  current_pose(1) += a * sin(heading);
  return current_pose;
}

// returns <speed, control_pose, arrived>
std::tuple<arma::vec, arma::vec, bool> eval_speed(robot::Parameters const& parameters, arma::vec const& current_pose,
                                                  arma::vec const& desired_pose, double stop_distance,
                                                  bool stop_condition = true,
                                                  arma::vec const& trajectory_speed = arma::vec({0.0, 0.0})) {
  if (current_pose.empty())
    return std::make_tuple(arma::vec({0.0, 0.0}), current_pose, false);

  auto heading = current_pose(2);
  auto a = parameters.center_offset;
  auto L = parameters.L;
  auto K = parameters.K;
  arma::vec control_pose = make_control_pose(current_pose, a);

  arma::vec error = desired_pose.subvec(0, 1) - control_pose.subvec(0, 1);
  if (stop_condition && arma::norm(error) < stop_distance)
    return std::make_tuple(arma::vec({0.0, 0.0}), control_pose, true);
  arma::mat invA = {{cos(heading), sin(heading)}, {-(1.0 / a) * sin(heading), (1.0 / a) * cos(heading)}};
  arma::vec C = trajectory_speed + L % tanh((K / L) % error);
  return std::make_tuple(invA * C, control_pose, false);
}

// returns <speed, control_pose, arrived>
std::tuple<arma::vec, arma::vec, bool> eval_rotspeed(robot::Parameters const& parameters, arma::vec const& current_pose,
                                                     double desired_heading, double stop_heading) {
  if (current_pose.empty())
    return std::make_tuple(arma::vec({0.0, 0.0}), current_pose, false);
  auto current_heading = current_pose(2);
  auto w_max = parameters.w_max;
  auto a = parameters.center_offset;
  arma::vec control_pose = make_control_pose(current_pose, a);

  auto error = desired_heading - current_heading;
  auto w = w_max * tanh(error);
  if (std::fabs(error) < stop_heading)
    return std::make_tuple(arma::vec({0.0, 0.0}), control_pose, true);
  return std::make_tuple(arma::vec({0.0, w}), control_pose, false);
}
*/
auto none(robot::Parameters const& parameters) {
  // auto a = parameters.center_offset;
  return [=](arma::vec const& current_pose) {
    /*
        return make_status(current_pose, make_control_pose(current_pose, a),
                           std::make_tuple(arma::vec({0.0, 0.0}), make_control_pose(current_pose, a), true));
        */
    RobotControllerProgress progress;
    return progress;
  };
}

auto final_position(robot::Parameters const& parameters, RobotTask const& robot_task) {
  auto task = robot_task.pose();
  auto desired_pose = pose_to_vec(task.goal());
  auto stop_distance = robot_task.allowed_error();

  return [=](arma::vec const& current_pose) {
    auto eval_output = eval_speed(parameters, current_pose, desired_pose, stop_distance);
    return make_status(current_pose, desired_pose, eval_output);
  };

  /*
  auto desired_point = robot_task.positions.at(0);
  auto stop_distance = robot_task.stop_distance;
  arma::vec desired_pose = point2vec::pose(desired_point);

  return [=](arma::vec const& current_pose) {
    auto eval_output = eval_speed(parameters, current_pose, desired_pose, stop_distance);
    return make_status(current_pose, desired_pose, eval_output);
  };
  */
  // return none(parameters);
}

auto final_heading(robot::Parameters const& parameters, RobotTask const& /*robot_task*/) {
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

auto trajectory(robot::Parameters const& parameters, RobotTask const& /*robot_task*/) {
  /*
  auto task_positions = robot_task.positions;
  auto task_speeds = robot_task.speeds;
  auto stop_distance = robot_task.stop_distance;

  return [ positions = task_positions, speeds = task_speeds, i = std::size_t(0), stop_distance,
           parameters ](arma::vec const& current_pose) mutable {
    if (i == positions.size()) {
      arma::vec desired_pose = point2vec::pose(positions.back());
      auto eval_output = eval_speed(parameters, current_pose, desired_pose, stop_distance);
      return make_status(current_pose, desired_pose, eval_output);
    }

    arma::vec desired_pose = point2vec::pose(positions[i]);
    arma::vec trajectory_speed = point2vec::speed(speeds[i]);
    i++;

    auto eval_output =
        eval_speed(parameters, current_pose, desired_pose, stop_distance, i == positions.size(), trajectory_speed);
    return make_status(current_pose, desired_pose, eval_output);
  };
  */
  return none(parameters);
}

auto path(robot::Parameters const& parameters, RobotTask const& /*robot_task*/) {
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