#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "adapt_mcl/likelihood_field.hpp"
#include "adapt_mcl/particle_filter.hpp"

using adapt_mcl::LikelihoodField;
using adapt_mcl::MotionModelParams;
using adapt_mcl::ParticleFilter;
using adapt_mcl::ParticleFilterParams;
using adapt_mcl::SoftEmParams;

// 4m x 4m room, resolution 0.05m (80x80 cells) with walls on all sides.
static LikelihoodField make_room_field() {
  LikelihoodField::MapInfo info;
  info.origin_x   = 0.0f;
  info.origin_y   = 0.0f;
  info.resolution = 0.05f;
  info.width      = 80;
  info.height     = 80;

  std::vector<int8_t> data(80 * 80, 0);
  for (int i = 0; i < 80; ++i) {
    data[0 * 80 + i]  = 100;  // bottom wall
    data[79 * 80 + i] = 100;  // top wall
    data[i * 80 + 0]  = 100;  // left wall
    data[i * 80 + 79] = 100;  // right wall
  }
  return LikelihoodField(info, data, 0.05f);
}

// Build 360-degree scan endpoints in base_link frame (sensor at origin, no offset).
// Ray cast from (px, py, ptheta) in map frame; store result in base_link frame.
static std::vector<std::array<float, 2>> make_room_endpoints(
    float px, float py, float ptheta, const LikelihoodField& field) {
  const int N = 180;
  std::vector<std::array<float, 2>> eps(N);
  for (int i = 0; i < N; ++i) {
    float phi = i * (2.0f * static_cast<float>(M_PI) / N);
    float r = 0.0f;
    while (r < 10.0f) {
      r += 0.02f;
      if (!field.is_free(px + r * std::cos(ptheta + phi),
                         py + r * std::sin(ptheta + phi))) break;
    }
    // Endpoint in base_link frame (sensor at origin, theta=0 heading).
    eps[i] = {r * std::cos(phi), r * std::sin(phi)};
  }
  return eps;
}

TEST(ParticleFilter, ConvergesFromTightInitialization) {
  auto field = make_room_field();

  ParticleFilterParams pf_params;
  pf_params.n_particles              = 500;
  pf_params.init_spread_pos_m        = 0.1f;
  pf_params.init_spread_angle_rad    = 0.05f;
  pf_params.init_random_fraction     = 0.0f;
  pf_params.ess_resample_threshold   = 0.5f;
  pf_params.ess_recovery_threshold   = 0.05f;

  MotionModelParams motion_params;
  motion_params.alpha1 = 0.02f;
  motion_params.alpha2 = 0.05f;
  motion_params.alpha3 = 0.02f;
  motion_params.alpha4 = 0.05f;

  SoftEmParams em_params;
  em_params.n_rays   = 20;
  em_params.em_iters = 2;

  ParticleFilter pf(pf_params, motion_params, em_params);

  // True pose.
  const float TRUE_X = 2.0f, TRUE_Y = 2.0f, TRUE_THETA = 0.3f;
  pf.initialize(field, TRUE_X, TRUE_Y, TRUE_THETA);

  auto eps = make_room_endpoints(TRUE_X, TRUE_Y, TRUE_THETA, field);

  // Static update: no odometry motion.
  float ox = 0.0f, oy = 0.0f, oth = 0.0f;
  float est_x, est_y, est_theta;

  for (int step = 0; step < 10; ++step) {
    auto [x, y, theta] = pf.update(field, eps, ox, oy, oth, ox, oy, oth);
    est_x = x; est_y = y; est_theta = theta;
  }

  EXPECT_NEAR(est_x, TRUE_X, 0.15f)    << "x not converged";
  EXPECT_NEAR(est_y, TRUE_Y, 0.15f)    << "y not converged";
  EXPECT_NEAR(est_theta, TRUE_THETA, 0.2f) << "theta not converged";
}

TEST(ParticleFilter, TracksStraightMotion) {
  auto field = make_room_field();

  ParticleFilterParams pf_params;
  pf_params.n_particles           = 500;
  pf_params.init_spread_pos_m     = 0.05f;
  pf_params.init_spread_angle_rad = 0.02f;
  pf_params.init_random_fraction  = 0.0f;

  MotionModelParams motion_params;
  SoftEmParams em_params;
  em_params.n_rays = 20;

  ParticleFilter pf(pf_params, motion_params, em_params);

  float rx = 1.0f, ry = 1.0f, rtheta = 0.0f;
  pf.initialize(field, rx, ry, rtheta);

  for (int step = 0; step < 15; ++step) {
    float nx = rx + 0.05f, ny = ry, ntheta = rtheta;
    auto eps = make_room_endpoints(nx, ny, ntheta, field);
    auto [ex, ey, et] = pf.update(field, eps, rx, ry, rtheta, nx, ny, ntheta);
    rx = nx; ry = ny; rtheta = ntheta;
    (void)ex; (void)ey; (void)et;
  }

  auto eps = make_room_endpoints(rx, ry, rtheta, field);
  auto [ex, ey, et] = pf.update(field, eps, rx, ry, rtheta, rx, ry, rtheta);

  EXPECT_NEAR(ex, rx, 0.15f) << "x tracking error too large";
  EXPECT_NEAR(ey, ry, 0.15f) << "y tracking error too large";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
