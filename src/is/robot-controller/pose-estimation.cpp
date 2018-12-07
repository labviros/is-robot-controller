
#include "pose-estimation.hpp"

namespace is {

void PoseEstimation::set_speed(is::common::Speed const& s) {
  speed = Eigen::VectorXd{2};
  speed << s.linear(), s.angular();
}

auto PoseEstimation::time_since_last_observation() const -> std::chrono::microseconds {
  return duration_cast<microseconds>(std::chrono::system_clock::now() - last_observation);
}

auto PoseEstimation::pose() -> is::common::Pose {
  using namespace std::chrono;
  time_since_observation = duration_cast<microseconds>(system_clock::now() - last_observation);
  auto estimative = kf.predict(speed);
  is::common::Pose pose;
  pose.mutable_position()->set_x(estimative(0));
  pose.mutable_position()->set_y(estimative(1));
  pose.mutable_orientation()->set_roll(estimative(2));
  return pose;
}

PoseEstimation::PoseEstimation() : speed(Eigen::VectorXd::Zero(2)) {
  kf.mean_post = Eigen::VectorXd::Zero(3);
  kf.covariance_post = Eigen::MatrixXd::Identity(3, 3);

  kf.system_noise_covariance = 1e-1 * Eigen::MatrixXd::Identity(3, 3);
  kf.system_model = [this](StateVector const& state, ActionVector const& action) -> StateVector {
    auto v = action(0);
    auto w = action(1);
    auto phi = state(2);
    auto dT = time_since_observation.count() / 1e6;
    auto update = StateVector{3};
    if (std::abs(w) > 1e-3) {
      update << v / w * (-std::sin(phi) + std::sin(phi + w * dT)),
          v / w * (std::cos(phi) - std::cos(phi + w * dT)), w * dT;
    } else {
      update << v * dT * std::cos(phi), v * dT * std::sin(phi), 0;
    }
    return state + update;
  };

  kf.system_jacobian = [this](StateVector const& state,
                              ActionVector const& action) -> JacobianMatrix {
    auto v = action(0);
    auto w = action(1);
    auto phi = state(2);
    auto dT = time_since_observation.count() / 1e6;
    auto jacobian = JacobianMatrix{3, 3};
    // clang-format off
    if (std::abs(w) > 1e-3) {
      jacobian << 1, 0, v / w * (-std::cos(phi) + std::cos(phi + w * dT)), 
                  0, 1, v / w * (-std::sin(phi) + std::sin(phi + w * dT)), 
                  0, 0, 1;
    } else {
      jacobian << 1, 0, -v*dT*std::sin(phi),
                  0, 1, +v*dT*std::cos(phi),
                  0, 0, 1;
    }
    // clang-format on
    return jacobian;
  };

  kf.observation_noise_covariance = 1e-1 * Eigen::MatrixXd::Identity(3, 3);
  kf.observation_model = [](StateVector const& state) -> ObservationVector { return state; };
  kf.observation_jacobian = [](StateVector const& state) -> JacobianMatrix {
    return Eigen::MatrixXd::Identity(3, 3);
  };
}

void PoseEstimation::run(is::Message const& message) {
  using namespace std::chrono;
  if (message.topic().find("FrameTransformation") != 0) { return; }

  auto frame_transformation = message.unpack<is::vision::FrameTransformation>();
  if (!frame_transformation || !frame_transformation->has_tf()) { return; }

  auto measure_time = message.has_created_at() ? message.created_at() : system_clock::now();
  if (measure_time < last_observation) {
    is::warn("event=PoseEstimation.OldObservation");
    return;
  }

  time_since_observation = duration_cast<microseconds>(measure_time - last_observation);
  last_observation = measure_time;

  auto tf = frame_transformation->tf().doubles();

  // TODO: Check sizes, dimensions, row/col major
  auto x = tf.Get(0 * 4 + 3);
  auto y = tf.Get(1 * 4 + 3);
  auto pitch = -std::asin(tf.Get(2 * 4 + 0));
  auto yaw = std::atan2(tf.Get(2 * 4 + 1) / cos(pitch), tf.Get(2 * 4 + 2) / cos(pitch));
  auto roll = std::atan2(tf.Get(1 * 4 + 0) / cos(pitch), tf.Get(0 * 4 + 0) / cos(pitch));

  auto measurement = StateVector{3};
  measurement << x, y, roll;
  kf.predict(speed);
  auto pose = kf.correct(measurement);

  on_new_measurement(message.topic());

  is::info("event=PoseEstimation.NewPose topic={} x={} y={} heading={}", message.topic(), pose(0),
           pose(1), 180 * pose(2) / 3.14);
}

}  // namespace is