#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

namespace adapt_mcl {

class LikelihoodField {
 public:
  struct MapInfo {
    float origin_x{0.0f};   // map origin in world [m]
    float origin_y{0.0f};
    float resolution{0.05f}; // meters per cell
    int width{0};            // cells
    int height{0};
  };

  /// Build likelihood field from an OccupancyGrid.
  /// @param info        Map metadata.
  /// @param data        Occupancy values: -1=unknown, 0=free, 100=occupied.
  /// @param sigma_hit   Gaussian std dev for the hit model [m].
  LikelihoodField(const MapInfo& info,
                  const std::vector<int8_t>& data,
                  float sigma_hit);

  /// Lookup precomputed likelihood p_hit at a world-frame endpoint.
  /// Returns 0 for out-of-bounds queries.
  float get_likelihood(float wx, float wy) const;

  /// True if (wx, wy) is a known-free cell (for particle injection).
  bool is_free(float wx, float wy) const;

  /// Sample a uniformly random free-space pose (x, y, theta).
  std::tuple<float, float, float> sample_free_pose(std::mt19937& rng) const;

  /// Refine a pose estimate by gradient ascent on the scan log-likelihood.
  /// Uses all provided base_link endpoints. Runs up to `iters` steps.
  std::tuple<float, float, float> refine_pose(
      float px, float py, float pt,
      const std::vector<std::array<float, 2>>& endpoints_bl,
      float alpha = 0.9f, float p_uniform = 0.033f,
      int iters = 8, float step = 0.003f) const;

  const MapInfo& info() const { return info_; }

 private:
  void build_distance_field(const std::vector<int8_t>& data);

  int to_cell_x(float wx) const;
  int to_cell_y(float wy) const;
  int cell_index(int cx, int cy) const { return cy * info_.width + cx; }
  bool in_bounds(int cx, int cy) const;

  MapInfo info_;
  float sigma_hit_;
  std::vector<float> likelihood_field_;  // precomputed Gaussian likelihood per cell
  std::vector<bool> free_mask_;          // true = free cell
  std::vector<int> free_cell_indices_;   // indices of free cells (for sampling)
};

}  // namespace adapt_mcl
