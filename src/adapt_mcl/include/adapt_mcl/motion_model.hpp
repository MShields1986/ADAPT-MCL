#pragma once

#include <random>
#include <vector>

#include "adapt_mcl/particle.hpp"

namespace adapt_mcl {

struct MotionModelParams {
  /// Omnidirectional motion model noise coefficients.
  /// alpha1: rotation noise from rotation
  /// alpha2: rotation noise from translation magnitude
  /// alpha3: forward translation noise from translation magnitude
  /// alpha4: forward translation noise from rotation
  /// alpha5: lateral translation noise from translation magnitude (omni lateral)
  float alpha1{0.05f};
  float alpha2{0.10f};
  float alpha3{0.05f};
  float alpha4{0.10f};
  float alpha5{0.05f};
  float min_trans_noise{0.0f};  // floor on translation stddev [m]
  float min_rot_noise{0.0f};    // floor on rotation stddev [rad]
};

class MotionModel {
 public:
  explicit MotionModel(const MotionModelParams& params);

  /// Apply odometry motion model to all particles in-place.
  /// Takes the odometry delta in the odometry frame.
  /// @param x_old, y_old, theta_old  Previous odometry pose.
  /// @param x_new, y_new, theta_new  Current odometry pose.
  void predict(std::vector<Particle>& particles,
               float x_old, float y_old, float theta_old,
               float x_new, float y_new, float theta_new,
               std::mt19937& rng) const;

 private:
  MotionModelParams params_;
};

/// Wrap angle to [-pi, pi].
float normalize_angle(float a);

}  // namespace adapt_mcl
