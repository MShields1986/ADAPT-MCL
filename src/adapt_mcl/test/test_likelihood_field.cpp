#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "adapt_mcl/likelihood_field.hpp"

using adapt_mcl::LikelihoodField;

// Build a tiny 10x10 map with one obstacle cell at (5, 5).
static LikelihoodField make_test_field(float sigma = 0.05f) {
  LikelihoodField::MapInfo info;
  info.origin_x   = 0.0f;
  info.origin_y   = 0.0f;
  info.resolution = 0.1f;
  info.width      = 10;
  info.height     = 10;

  std::vector<int8_t> data(100, 0);  // all free
  data[5 * info.width + 5] = 100;    // obstacle at cell (5,5)

  return LikelihoodField(info, data, sigma);
}

TEST(LikelihoodField, PeakAtObstacle) {
  auto field = make_test_field(0.05f);
  // World position of obstacle center: x=5.5*0.1=0.55, y=5.5*0.1=0.55
  float l_at_obstacle  = field.get_likelihood(0.55f, 0.55f);
  float l_far          = field.get_likelihood(0.0f, 0.0f);
  EXPECT_NEAR(l_at_obstacle, 1.0f, 0.05f)  // distance ≈ 0 → likelihood ≈ 1
      << "likelihood at obstacle center should be near 1";
  EXPECT_LT(l_far, l_at_obstacle)
      << "likelihood should decrease away from obstacle";
}

TEST(LikelihoodField, DecaysWithDistance) {
  auto field = make_test_field(0.10f);
  // Move one cell away (0.1 m): expected likelihood = exp(-0.1²/(2*0.1²)) = exp(-0.5) ≈ 0.607
  float l_one_cell = field.get_likelihood(0.65f, 0.55f);  // one cell right of obstacle
  EXPECT_NEAR(l_one_cell, std::exp(-0.5f), 0.15f);
}

TEST(LikelihoodField, OutOfBoundsReturnsZero) {
  auto field = make_test_field();
  EXPECT_EQ(field.get_likelihood(-1.0f, -1.0f), 0.0f);
  EXPECT_EQ(field.get_likelihood(999.0f, 999.0f), 0.0f);
}

TEST(LikelihoodField, FreeMaskCorrect) {
  LikelihoodField::MapInfo info;
  info.origin_x   = 0.0f;
  info.origin_y   = 0.0f;
  info.resolution = 0.1f;
  info.width      = 5;
  info.height     = 5;

  std::vector<int8_t> data(25, 0);
  data[2 * 5 + 2] = 100;  // obstacle at (2,2)

  LikelihoodField field(info, data, 0.05f);

  // Cell (0,0) is free.
  EXPECT_TRUE(field.is_free(0.05f, 0.05f));
  // Cell (2,2) is occupied.
  EXPECT_FALSE(field.is_free(0.25f, 0.25f));
}

TEST(LikelihoodField, SampleFreeReturnsFreeCell) {
  auto field = make_test_field();
  std::mt19937 rng(42);
  for (int i = 0; i < 20; ++i) {
    auto [x, y, theta] = field.sample_free_pose(rng);
    EXPECT_TRUE(field.is_free(x, y))
        << "sampled pose (" << x << ", " << y << ") is not in free space";
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
