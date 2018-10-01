
#include "extended-kalman-filter.hpp"

namespace is {

auto ExtendedKalmanFilter::predict(ActionVector const& action) -> StateVector const& {
  mean_prior = system_model(mean_post, action);
  auto jacobian = system_jacobian(mean_post, action);
  covariance_prior = jacobian * covariance_post * jacobian.transpose() + system_noise_covariance;
  return mean_prior;
}

auto ExtendedKalmanFilter::correct(ObservationVector const& observation) -> StateVector const& {
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

}