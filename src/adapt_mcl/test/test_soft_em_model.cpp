#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <vector>
#include "adapt_mcl/likelihood_field.hpp"
#include "adapt_mcl/particle.hpp"
#include "adapt_mcl/soft_em_model.hpp"

using adapt_mcl::LikelihoodField;
using adapt_mcl::Particle;
using adapt_mcl::SoftEmModel;
using adapt_mcl::SoftEmParams;

// 1m x 1m map with a right wall at x=0.95m (cell 19 of 20).
static LikelihoodField make_wall_field() {
  LikelihoodField::MapInfo info;
  info.origin_x = info.origin_y = 0.0f;
  info.resolution = 0.05f;
  info.width = info.height = 20;
  std::vector<int8_t> data(400, 0);
  for (int r = 0; r < 20; ++r) data[r * 20 + 19] = 100;
  return LikelihoodField(info, data, 0.05f);
}

// Build endpoints_bl: sensor at base_link origin, all rays pointing right at given range.
static std::vector<std::array<float, 2>> make_endpoints(int n, float range, float angle) {
  std::vector<std::array<float, 2>> eps(n);
  for (auto& ep : eps) {
    ep[0] = range * std::cos(angle);
    ep[1] = range * std::sin(angle);
  }
  return eps;
}

TEST(SoftEmModel, HighAlphaForInlierParticle) {
  auto field = make_wall_field();
  SoftEmParams params;
  params.n_rays = 30; params.em_iters = 3;
  params.alpha_prior = 8.0f; params.beta_prior = 2.0f;
  SoftEmModel model(params);

  std::vector<Particle> particles(1);
  particles[0] = {0.5f, 0.5f, 0.0f, 0.0f, 0.9f};

  // Sensor at origin, range 0.45m right → endpoint (0.45, 0) in base_link.
  // With particle at (0.5, 0.5, 0): map endpoint = (0.95, 0.5) → on right wall.
  auto eps = make_endpoints(30, 0.45f, 0.0f);
  model.compute_weights(particles, eps, field);
  EXPECT_GT(particles[0].alpha, 0.7f)
      << "alpha should be high for correct-pose particle";
}

TEST(SoftEmModel, InlierParticleHigherWeightThanOutlier) {
  auto field = make_wall_field();
  SoftEmParams params;
  params.n_rays = 20; params.em_iters = 2;
  SoftEmModel model(params);

  std::vector<Particle> particles(2);
  particles[0] = {0.5f, 0.5f, 0.0f, 0.0f, 0.9f};  // map endpoint = (0.95, 0.5): on wall
  particles[1] = {0.1f, 0.1f, 0.0f, 0.0f, 0.9f};  // map endpoint = (0.55, 0.1): mid-air

  auto eps = make_endpoints(20, 0.45f, 0.0f);
  model.compute_weights(particles, eps, field);
  EXPECT_GT(particles[0].log_weight, particles[1].log_weight)
      << "correct-pose particle should have higher log weight";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
