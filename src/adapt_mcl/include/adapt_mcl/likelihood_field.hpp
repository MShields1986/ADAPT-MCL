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
  /// @param cpd_sigma   CPD Gaussian bandwidth [m] (0 = disabled).
  /// @param cpd_w       CPD uniform outlier weight in [0,1] (ignored if cpd_sigma=0).
  LikelihoodField(const MapInfo& info,
                  const std::vector<int8_t>& data,
                  float sigma_hit,
                  float cpd_sigma = 0.0f,
                  float cpd_w = 0.0f);

  /// Lookup precomputed likelihood p_hit at a world-frame endpoint.
  /// Returns 0 for out-of-bounds queries.
  float get_likelihood(float wx, float wy) const;

  /// Lookup precomputed CPD likelihood at a world-frame endpoint.
  /// Returns (1-w)*exp(-d²/2σ²_cpd) + w, in [w, 1].
  /// Falls back to get_likelihood() if CPD field was not built (cpd_sigma=0).
  float get_cpd_likelihood(float wx, float wy) const;

  /// True if (wx, wy) is a known-free cell (for particle injection).
  bool is_free(float wx, float wy) const;

  /// True if (wx, wy) is a known-occupied cell (data > 50).
  /// Out-of-bounds and unknown cells return false (no penalty — map may be incomplete).
  bool is_occupied(float wx, float wy) const {
    int cx = to_cell_x(wx); int cy = to_cell_y(wy);
    if (!in_bounds(cx, cy)) return false;
    return occupied_mask_[cell_index(cx, cy)];
  }

  /// Sample a uniformly random free-space pose (x, y, theta).
  std::tuple<float, float, float> sample_free_pose(std::mt19937& rng) const;

  /// Candidate pose with associated scan log-score for global localization.
  struct PoseCandidate {
    float x, y, theta;
    float log_score;   // Stage 1: likelihood-field score (higher = better)
    float caer_score;  // Stage 2: inlier fraction (higher = better; 0.0 if not computed)
  };

  /// Score a coarse grid of poses against a scan and return the top-K candidates.
  /// @param endpoints_bl  Ray endpoints in base_link frame.
  /// @param pos_step_m    Spatial grid stride [m].
  /// @param angle_bins    Number of heading bins (full 2π divided equally).
  /// @param top_k         How many candidates to return (sorted descending by score).
  /// @param n_rays        Number of rays to subsample for scoring.
  std::vector<PoseCandidate> scan_match_candidates(
      const std::vector<std::array<float, 2>>& endpoints_bl,
      float pos_step_m,
      int   angle_bins,
      int   top_k,
      int   n_rays) const;

  /// Same as scan_match_candidates but uses CPD GMM scoring on the full grid
  /// instead of the precomputed LF Gaussians.  This means the outlier-aware
  /// score is applied to every grid cell — not just the LF top-N — so the true
  /// pose is never pre-filtered out before CPD sees it.
  std::vector<PoseCandidate> scan_match_candidates_cpd(
      const std::vector<std::array<float, 2>>& endpoints_bl,
      float pos_step_m,
      int   angle_bins,
      int   top_k,
      int   n_rays,
      float cpd_sigma,
      float cpd_w) const;

  /// Re-rank candidates by inlier-fraction CAER (higher inlier fraction = better).
  /// Uses DDA raycasting to check geometric consistency with the map.
  /// @param candidates       Candidates to re-rank (modified in-place, returned sorted).
  /// @param endpoints_bl     Ray endpoints in base_link frame.
  /// @param n_rays           Number of rays to subsample for CAER scoring.
  /// @param inlier_threshold |r_expected - r_actual| < thresh counts as inlier [m].
  std::vector<PoseCandidate> caer_rerank(
      std::vector<PoseCandidate> candidates,
      const std::vector<std::array<float, 2>>& endpoints_bl,
      int   n_rays,
      float inlier_threshold) const;

  /// Re-rank candidates using a CPD-style GMM log-likelihood (outlier-aware scoring).
  /// Uses the raw distance field with an explicit uniform outlier component so that
  /// scan points that fall in map-changed regions don't unduly penalise the true pose.
  ///
  /// Score per ray: log((1-w)/M * gauss(d; 0, sigma) + w/area)
  ///   where d = nearest-obstacle distance, M = occupied cell count, area = map area.
  ///
  /// @param candidates    Candidates to re-rank (log_score overwritten with CPD score).
  /// @param endpoints_bl  Ray endpoints in base_link frame.
  /// @param n_rays        Number of rays to subsample.
  /// @param cpd_sigma     Gaussian bandwidth [m] (independent of sigma_hit; use ~0.10m).
  /// @param cpd_w         Uniform outlier weight in [0,1] (use ~0.10–0.30).
  std::vector<PoseCandidate> cpd_rerank(
      std::vector<PoseCandidate> candidates,
      const std::vector<std::array<float, 2>>& endpoints_bl,
      int   n_rays,
      float cpd_sigma,
      float cpd_w) const;

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

  /// DDA raycasting: returns distance to first occupied cell along (dir_x, dir_y),
  /// capped at max_range. (dir_x, dir_y) should be a unit vector.
  float raycast(float px, float py, float dir_x, float dir_y, float max_range) const;

  int to_cell_x(float wx) const;
  int to_cell_y(float wy) const;
  int cell_index(int cx, int cy) const { return cy * info_.width + cx; }
  bool in_bounds(int cx, int cy) const;

  MapInfo info_;
  float sigma_hit_;
  float cpd_sigma_{0.0f};
  float cpd_w_{0.0f};
  std::vector<float> likelihood_field_;  // precomputed Gaussian likelihood per cell
  std::vector<float> cpd_field_;         // precomputed CPD likelihood: (1-w)*exp(-d²/2σ²_cpd)+w
  std::vector<float> dist_field_;        // raw BFS distance to nearest occupied cell [m]
  std::vector<bool> free_mask_;          // true = free cell
  std::vector<bool> occupied_mask_;      // true = occupied cell (data[i] > 50)
  std::vector<int> free_cell_indices_;   // indices of free cells (for sampling)
  int   occupied_cell_count_{0};         // number of occupied cells (for CPD density)
  float map_area_{1.0f};                 // map bounding-box area [m²]
  float free_space_area_{1.0f};          // free-cell area [m²] (for CPD uniform outlier term)
};

}  // namespace adapt_mcl
