#include "adapt_mcl/motion_model.hpp"

#include <cmath>

namespace adapt_mcl {

float normalize_angle(float a) {
  while (a > M_PIf32) a -= 2.0f * M_PIf32;
  while (a < -M_PIf32) a += 2.0f * M_PIf32;
  return a;
}

MotionModel::MotionModel(const MotionModelParams& params) : params_(params) {}

void MotionModel::predict(std::vector<Particle>& particles,
                          float x_old, float y_old, float theta_old,
                          float x_new, float y_new, float theta_new,
                          std::mt19937& rng) const {
  // Omnidirectional motion model.
  //
  // Decompose the odom delta into body-frame components so that lateral
  // motion (which the Dingo-O uses freely) does not inflate rotational noise
  // the way the diff-drive decomposition does.
  //
  //   delta_fwd   : forward displacement in the robot's body frame
  //   delta_lat   : lateral displacement (+ = left)
  //   delta_theta : heading change
  //
  // Noise model (independent Gaussians):
  //   sigma_theta = sqrt(alpha1 * delta_theta^2 + alpha2 * dist^2)
  //   sigma_fwd   = sqrt(alpha3 * dist^2       + alpha4 * delta_theta^2)
  //   sigma_lat   = sqrt(alpha5 * dist^2       + alpha4 * delta_theta^2)

  const float dx     = x_new - x_old;
  const float dy     = y_new - y_old;
  const float dtheta = normalize_angle(theta_new - theta_old);

  // Rotate odom delta into body frame at theta_old.
  const float cos_h = std::cos(theta_old);
  const float sin_h = std::sin(theta_old);
  const float delta_fwd = dx * cos_h + dy * sin_h;
  const float delta_lat = -dx * sin_h + dy * cos_h;
  const float dist2 = delta_fwd * delta_fwd + delta_lat * delta_lat;

  const float a1 = params_.alpha1;
  const float a2 = params_.alpha2;
  const float a3 = params_.alpha3;
  const float a4 = params_.alpha4;
  const float a5 = params_.alpha5;

  float var_theta = a1 * dtheta * dtheta + a2 * dist2;
  float var_fwd   = a3 * dist2 + a4 * dtheta * dtheta;
  float var_lat   = a5 * dist2 + a4 * dtheta * dtheta;

  float min_t2 = params_.min_trans_noise * params_.min_trans_noise;
  float min_r2 = params_.min_rot_noise   * params_.min_rot_noise;
  var_theta = std::max(var_theta, min_r2 + 1e-12f);
  var_fwd   = std::max(var_fwd,   min_t2 + 1e-12f);
  var_lat   = std::max(var_lat,   min_t2 + 1e-12f);

  std::normal_distribution<float> n_theta(0.0f, std::sqrt(var_theta));
  std::normal_distribution<float> n_fwd  (0.0f, std::sqrt(var_fwd));
  std::normal_distribution<float> n_lat  (0.0f, std::sqrt(var_lat));

  for (auto& p : particles) {
    const float fwd = delta_fwd + n_fwd(rng);
    const float lat = delta_lat + n_lat(rng);
    const float dth = dtheta   + n_theta(rng);

    // Apply displacement in particle's body frame.
    const float cp = std::cos(p.theta);
    const float sp = std::sin(p.theta);
    p.x     += fwd * cp - lat * sp;
    p.y     += fwd * sp + lat * cp;
    p.theta  = normalize_angle(p.theta + dth);
  }
}

}  // namespace adapt_mcl
