#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "adapt_mcl/likelihood_field.hpp"
#include "adapt_mcl/motion_model.hpp"
#include "adapt_mcl/particle.hpp"
#include "adapt_mcl/soft_em_model.hpp"

namespace adapt_mcl {

struct ParticleFilterParams {
  int n_particles{1000};
  float ess_resample_threshold{0.5f};  // resample if ESS/N < this
  float ess_recovery_threshold{0.1f};  // inject recovery particles if ESS/N < this
  float recovery_fraction{0.2f};       // fraction of particles to replace on recovery
  float roughening_pos_m{0.005f};      // position roughening after resample [m]
  float roughening_angle_rad{0.01f};   // angle roughening after resample [rad]
  float init_spread_pos_m{0.1f};       // initial position spread [m]
  float init_spread_angle_rad{0.1f};   // initial angle spread [rad]
  float init_random_fraction{0.05f};   // fraction of particles placed randomly in free space
  // KLD-sampling (Fox 2001) — adaptive particle count
  bool  use_kld_sampling{false};
  float kld_bin_size_m{0.2f};       // position bin width [m]
  float kld_bin_size_rad{0.2f};     // heading bin width [rad] (~11.5°)
  float kld_epsilon{0.05f};         // max KL-divergence bound
  float kld_delta{0.01f};           // failure probability (z=2.326 hardcoded)
  int   kld_min_particles{200};     // floor on particle count after resample
  int   kld_max_particles{5000};    // ceiling on particle count (also initial N when KLD on)
  // MAP estimate: return highest-weight pre-resample particle instead of post-resample mean
  bool  use_map_estimate{false};
  // w_slow/w_fast adaptive recovery (Fox 2003 augmented MCL)
  bool  use_wslow_wfast{false};
  float wslow_alpha{0.001f};           // slow EMA decay (time const ≈ 1000 updates)
  float wfast_alpha{0.1f};            // fast EMA decay (time const ≈ 10 updates)
  float wfast_wslow_threshold{0.5f};  // inject when w_fast/w_slow < this
  // Global-init phase: augmented MCL injection (step-count triggered)
  int   global_init_steps{0};              // # updates to inject; 0 = disabled
  float global_init_injection_frac{0.05f}; // fraction of particles replaced per step
  // Temperature annealing: scale log-weights by beta for first N scans after init.
  // Prevents instant collapse to aliased hypothesis during global localization.
  // beta ramps linearly from init_temperature → 1.0 over init_temperature_steps scans.
  int   init_temperature_steps{0};   // 0 = disabled
  float init_temperature{0.05f};     // starting beta (e.g. 0.05 = 5% of full weight update)
  // Scan-match seed for global localization
  bool  use_scan_match_seed{false};
  float scan_match_pos_step_m{0.4f};       // coarse grid spacing [m]
  int   scan_match_angle_bins{48};         // 48 → 7.5° steps
  int   scan_match_top_k{20};             // candidates to seed around (final top-K after CAER)
  float scan_match_candidate_frac{0.5f};  // fraction of N seeded (rest = uniform)
  // CAER re-ranking: two-stage scoring for global localization
  bool  use_caer_rerank{false};
  int   caer_rerank_top_n{200};        // Stage 1 → top-N for CAER re-ranking
  int   caer_rerank_n_rays{200};       // rays to use in CAER scoring
  float caer_inlier_threshold{0.30f};  // |r_expected - r_actual| < thresh = inlier [m]
  // CPD re-ranking: outlier-aware GMM scoring for global localization
  // Helps when map has changed (map-change regions treated as outliers, not penalised).
  bool  use_cpd_rerank{false};
  int   cpd_rerank_top_n{200};   // LF/CAER top-N passed into CPD re-ranking
  int   cpd_rerank_n_rays{200};  // rays per candidate in CPD scoring
  float cpd_sigma{0.10f};        // CPD Gaussian bandwidth [m]
  float cpd_w{0.20f};            // CPD uniform outlier weight [0,1]
};

class ParticleFilter {
 public:
  ParticleFilter(const ParticleFilterParams& pf_params,
                 const MotionModelParams& motion_params,
                 const SoftEmParams& em_params);

  /// Initialize particles around a known pose with a small Gaussian spread.
  /// A fraction init_random_fraction are placed uniformly in free space.
  void initialize(const LikelihoodField& field,
                  float x, float y, float theta);

  /// Global localization init: score a coarse CPD/LF grid, return the top candidate
  /// pose (x, y, theta). Call initialize_tracking() afterwards to commit to it.
  /// Returns (0, 0, 0) if no candidates found.
  std::tuple<float, float, float> initialize_global(
      const LikelihoodField& field,
      const std::vector<std::array<float, 2>>& endpoints_bl);

  /// Multi-hypothesis seeding from external candidate poses (e.g. VPR).
  /// Distributes n_particles across candidates as Gaussian clusters,
  /// weighted by the candidate scores. Each cluster uses
  /// init_spread_pos_m / init_spread_angle_rad as its spread.
  struct ExternalCandidate {
    float x, y, theta;
    float weight;  // relative importance (will be normalized)
  };
  void initialize_from_candidates(
      const std::vector<ExternalCandidate>& candidates);

  /// Switch to tracking mode around a known pose (e.g. the result of initialize_global).
  /// Spreads n_particles tightly using init_spread_pos_m / init_spread_angle_rad.
  /// No random fraction — all particles are near (x, y, theta).
  void initialize_tracking(const LikelihoodField& field,
                           float x, float y, float theta);

  /// Process one update step: motion prediction + sensor weighting + resampling.
  /// @param field          Precomputed likelihood field.
  /// @param endpoints_bl   Ray endpoints in base_link frame [m] (all sensors merged).
  /// @param x/y/theta_odom_old/new  Previous and current odometry poses.
  /// @return (x, y, theta) weighted mean pose estimate.
  std::tuple<float, float, float> update(
      const LikelihoodField& field,
      const std::vector<std::array<float, 2>>& endpoints_bl,
      float x_odom_old, float y_odom_old, float theta_odom_old,
      float x_odom_new, float y_odom_new, float theta_odom_new);

  const std::vector<Particle>& particles() const { return particles_; }
  bool is_initialized() const { return initialized_; }

  /// Candidates from the most recent initialize_global() call.
  /// Useful for trajectory-consistent re-scoring after N scans of robot motion.
  const std::vector<LikelihoodField::PoseCandidate>& last_global_candidates() const {
    return last_global_candidates_;
  }

 private:
  ParticleFilterParams pf_params_;
  MotionModel motion_model_;
  SoftEmModel sensor_model_;
  std::vector<Particle> particles_;
  std::mt19937 rng_;
  bool initialized_{false};
  int debug_counter_{0};
  int scan_step_{0};  // counts update() calls for global-init phase

  // w_slow/w_fast state
  double log_w_slow_{0.0};
  double log_w_fast_{0.0};
  bool   wslow_initialized_{false};

  float compute_ess() const;
  void resample_systematic(const LikelihoodField& field);
  void resample_kld(const LikelihoodField& field);
  void build_kld_table();
  void add_roughening();
  std::tuple<float, float, float> weighted_mean_pose() const;

  // Inject n lowest-weight particles with fresh free-space samples.
  void inject_random_particles(const LikelihoodField& field, int n);

  // Normalize log weights to sum-to-one linear weights in-place.
  // Returns log_Z = logsumexp(log_w_i).
  double normalize_log_weights();

  std::vector<int> kld_table_;  // kld_table_[k] = required N for k occupied bins
  std::vector<LikelihoodField::PoseCandidate> last_global_candidates_;
};

}  // namespace adapt_mcl
