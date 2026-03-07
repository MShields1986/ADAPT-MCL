#pragma once

#include <array>
#include <vector>

#include "adapt_mcl/likelihood_field.hpp"
#include "adapt_mcl/particle.hpp"

namespace adapt_mcl {

struct SoftEmParams {
  int n_rays{30};            // stratified subsampling count (across all endpoints)
  float p_uniform{0.033f};  // uniform outlier density (1/z_max, ~1/30m)
  float alpha_prior{8.0f};  // Beta(alpha_prior, beta_prior) prior on inlier fraction
  float beta_prior{2.0f};   // biased toward high inlier fraction
  int em_iters{2};           // EM iterations per particle per update
  float norm_exponent{0.5f}; // exponent for n_valid normalization (0.5 = sqrt)
  bool  use_z_short{false};
  float lambda_short{2.0f};   // exponential decay rate [1/m]
  float gamma_prior{1.0f};    // pseudo-count prior for short-reading fraction
};

/// Per-particle soft EM sensor model.
///
/// Input: ray endpoints pre-computed in the robot base_link frame.
/// The node is responsible for transforming sensor-frame rays (from each
/// LiDAR) to base_link using the static TF.  This cleanly handles any
/// number of sensors without changes to the PF core.
///
/// For a particle at (px, py, ptheta), the endpoint in map frame is:
///   ex_map = px + cos(ptheta)*ex_bl - sin(ptheta)*ey_bl
///   ey_map = py + sin(ptheta)*ex_bl + cos(ptheta)*ey_bl
class SoftEmModel {
 public:
  explicit SoftEmModel(const SoftEmParams& params);

  /// Compute and accumulate log weights for all particles.
  /// Stratified subsampling selects n_rays endpoints from endpoints_bl.
  /// Updates particle.log_weight and particle.alpha in-place.
  ///
  /// @param particles     Particle set (modified in-place).
  /// @param endpoints_bl  Ray endpoints in base_link frame [m].
  /// @param field         Precomputed likelihood field.
  void compute_weights(std::vector<Particle>& particles,
                       const std::vector<std::array<float, 2>>& endpoints_bl,
                       const LikelihoodField& field) const;

 private:
  SoftEmParams params_;

  float em_estimate_alpha(const float* likelihoods, int n) const;
  std::pair<float,float> em_estimate_alpha_gamma(
      const float* likelihoods, const float* ranges, int n) const;
};

}  // namespace adapt_mcl
