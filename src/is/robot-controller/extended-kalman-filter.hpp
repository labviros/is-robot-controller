
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

  auto predict(ActionVector const& action) -> StateVector const&;
  auto correct(ObservationVector const& observation) -> StateVector const&;
};

}  // namespace is