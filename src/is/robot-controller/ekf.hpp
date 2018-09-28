#pragma once

#include <Eigen/Dense>
#include <functional>

using StateVector = Eigen::VectorXd;
using ActionVector = Eigen::VectorXd;
using ObservationVector = Eigen::VectorXd;
using JacobianMatrix = Eigen::MatrixXd;

namespace is {

struct ExtendedKalmanFilter {
  Eigen::VectorXd mean_prior;
  Eigen::MatrixXd system_noise_covariance;
  Eigen::MatrixXd covariance_prior;

  Eigen::VectorXd mean_post;
  Eigen::MatrixXd observation_noise_covariance;
  Eigen::MatrixXd covariance_post;

  std::function<StateVector(StateVector const&, ActionVector const&)> system_model;
  std::function<JacobianMatrix(StateVector const&, ActionVector const&)> system_jacobian;

  std::function<ObservationVector(StateVector const&)> observation_model;
  std::function<JacobianMatrix(StateVector const&)> observation_jacobian;

  ExtendedKalmanFilter() = default;

  auto predict(ActionVector const& action) -> StateVector const& {
    mean_prior = system_model(mean_post, action);
    auto jacobian = system_jacobian(mean_post, action);
    covariance_prior = jacobian * covariance_post * jacobian.transpose() + system_noise_covariance;
    return mean_prior;
  }

  auto correct(ObservationVector const& observation) -> StateVector const& {
    auto jacobian = observation_jacobian(observation);
    auto gain_denominator =
        jacobian * covariance_prior * jacobian.transpose() + observation_noise_covariance;
    auto kalman_gain = covariance_prior * jacobian.transpose() * gain_denominator.inverse();

    mean_post = mean_prior + kalman_gain * (observation - observation_model(mean_prior));
    covariance_post =
        (Eigen::MatrixXd::Identity(kalman_gain.rows(), jacobian.cols()) - kalman_gain * jacobian) *
        covariance_prior;
    return mean_post;
  }
};

}