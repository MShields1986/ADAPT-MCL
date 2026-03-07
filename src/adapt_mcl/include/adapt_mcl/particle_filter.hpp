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

 private:
  ParticleFilterParams pf_params_;
  MotionModel motion_model_;
  SoftEmModel sensor_model_;
  std::vector<Particle> particles_;
  std::mt19937 rng_;
  bool initialized_{false};
  int debug_counter_{0};

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

  // Normalize log weights to sum-to-one linear weights in-place.
  // Returns log_Z = logsumexp(log_w_i).
  double normalize_log_weights();

  std::vector<int> kld_table_;  // kld_table_[k] = required N for k occupied bins
};

}  // namespace adapt_mcl
