#include "adapt_mcl/particle_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "adapt_mcl/motion_model.hpp"

namespace adapt_mcl {

ParticleFilter::ParticleFilter(const ParticleFilterParams& pf_params,
                               const MotionModelParams& motion_params,
                               const SoftEmParams& em_params)
    : pf_params_(pf_params),
      motion_model_(motion_params),
      sensor_model_(em_params),
      rng_(std::random_device{}()) {
  if (pf_params_.use_kld_sampling) build_kld_table();
}

void ParticleFilter::initialize(const LikelihoodField& field,
                                float x, float y, float theta) {
  int init_n = pf_params_.use_kld_sampling ? pf_params_.kld_max_particles
                                           : pf_params_.n_particles;
  particles_.resize(init_n);

  int n_random = static_cast<int>(init_n * pf_params_.init_random_fraction);
  int n_near   = init_n - n_random;

  std::normal_distribution<float> pos_noise(0.0f, pf_params_.init_spread_pos_m);
  std::normal_distribution<float> ang_noise(0.0f, pf_params_.init_spread_angle_rad);

  for (int i = 0; i < n_near; ++i) {
    particles_[i].x          = x + pos_noise(rng_);
    particles_[i].y          = y + pos_noise(rng_);
    particles_[i].theta      = normalize_angle(theta + ang_noise(rng_));
    particles_[i].log_weight = 0.0f;
    particles_[i].alpha      = 0.9f;
  }

  for (int i = n_near; i < init_n; ++i) {
    auto [rx, ry, rtheta] = field.sample_free_pose(rng_);
    particles_[i].x          = rx;
    particles_[i].y          = ry;
    particles_[i].theta      = rtheta;
    particles_[i].log_weight = 0.0f;
    particles_[i].alpha      = 0.9f;
  }

  initialized_ = true;
}

std::tuple<float, float, float> ParticleFilter::update(
    const LikelihoodField& field,
    const std::vector<std::array<float, 2>>& endpoints_bl,
    float x_odom_old, float y_odom_old, float theta_odom_old,
    float x_odom_new, float y_odom_new, float theta_odom_new) {
  if (!initialized_) {
    throw std::runtime_error("ParticleFilter: call initialize() before update()");
  }

  // Reset log weights before weighting.
  for (auto& p : particles_) {
    p.log_weight = 0.0f;
  }

  // 1. Motion update (predict).
  motion_model_.predict(particles_,
                        x_odom_old, y_odom_old, theta_odom_old,
                        x_odom_new, y_odom_new, theta_odom_new,
                        rng_);

  // 2. Sensor update (weight).
  sensor_model_.compute_weights(particles_, endpoints_bl, field);

  // 3. Normalize weights.
  double log_Z = normalize_log_weights();
  double log_avg_w = log_Z - std::log(static_cast<double>(particles_.size()));

  // 3b. w_slow/w_fast adaptive recovery (Fox 2003 augmented MCL).
  if (pf_params_.use_wslow_wfast) {
    if (!wslow_initialized_) {
      log_w_slow_ = log_avg_w;
      log_w_fast_ = log_avg_w;
      wslow_initialized_ = true;
    } else {
      log_w_slow_ += pf_params_.wslow_alpha * (log_avg_w - log_w_slow_);
      log_w_fast_ += pf_params_.wfast_alpha * (log_avg_w - log_w_fast_);
    }

    double ratio = std::exp(log_w_fast_ - log_w_slow_);
    double p_inject = std::max(0.0, 1.0 - ratio / pf_params_.wfast_wslow_threshold);
    int n_inject = static_cast<int>(p_inject * static_cast<double>(particles_.size()));

    if (n_inject > 0) {
      std::vector<int> order(particles_.size());
      std::iota(order.begin(), order.end(), 0);
      std::partial_sort(order.begin(), order.begin() + n_inject, order.end(),
                        [&](int a, int b) {
                          return particles_[a].log_weight < particles_[b].log_weight;
                        });
      const double ulw = -std::log(static_cast<double>(particles_.size()));
      for (int i = 0; i < n_inject; ++i) {
        auto [rx, ry, rtheta] = field.sample_free_pose(rng_);
        auto& p = particles_[order[i]];
        p.x = rx; p.y = ry; p.theta = rtheta; p.log_weight = ulw;
      }
      log_Z = normalize_log_weights();
      log_avg_w = log_Z - std::log(static_cast<double>(particles_.size()));
      log_w_fast_ += pf_params_.wfast_alpha * (log_avg_w - log_w_fast_);
    }
  }

  // 3c. Cache MAP pose (highest log-weight particle) before resampling.
  float map_x = 0.0f, map_y = 0.0f, map_theta = 0.0f;
  if (pf_params_.use_map_estimate) {
    auto best = std::max_element(particles_.begin(), particles_.end(),
        [](const Particle& a, const Particle& b) {
          return a.log_weight < b.log_weight;
        });
    map_x = best->x; map_y = best->y; map_theta = best->theta;
  }

  // 4. Compute ESS.
  float ess = compute_ess();
  float ess_fraction = ess / static_cast<float>(particles_.size());

  // 5. Recovery injection: replace a fraction of worst particles with random.
  if (ess_fraction < pf_params_.ess_recovery_threshold) {
    int n_inject = static_cast<int>(
        particles_.size() * pf_params_.recovery_fraction);
    // Sort particles by weight (ascending) and replace the lowest.
    std::vector<int> order(particles_.size());
    std::iota(order.begin(), order.end(), 0);
    std::partial_sort(order.begin(), order.begin() + n_inject, order.end(),
                      [&](int a, int b) {
                        return particles_[a].log_weight < particles_[b].log_weight;
                      });
    // Use uniform weight (log(1/N)) so injected particles don't dominate.
    const double uniform_log = -std::log(static_cast<double>(particles_.size()));
    for (int i = 0; i < n_inject; ++i) {
      auto [rx, ry, rtheta] = field.sample_free_pose(rng_);
      auto& p = particles_[order[i]];
      p.x = rx; p.y = ry; p.theta = rtheta;
      p.log_weight = uniform_log;
    }
    (void)normalize_log_weights();
    ess = compute_ess();
    ess_fraction = ess / static_cast<float>(particles_.size());
  }

  // 6. ESS-gated resampling + roughening.
  if (ess_fraction < pf_params_.ess_resample_threshold) {
    if (pf_params_.use_kld_sampling)
      resample_kld(field);
    else
      resample_systematic(field);
    add_roughening();
    // After resampling, weights are uniform.
    double uniform_log_w = -std::log(static_cast<double>(particles_.size()));
    for (auto& p : particles_) {
      p.log_weight = uniform_log_w;
    }
  }

  if (pf_params_.use_map_estimate) {
    return {map_x, map_y, map_theta};
  }
  return weighted_mean_pose();
}

float ParticleFilter::compute_ess() const {
  // ESS = 1 / sum(w_i^2) where w_i are linear weights (not log).
  double sum_sq = 0.0;
  for (const auto& p : particles_) {
    double w = std::exp(p.log_weight);
    sum_sq += w * w;
  }
  return static_cast<float>((sum_sq > 1e-300) ? (1.0 / sum_sq) : 0.0);
}

void ParticleFilter::resample_systematic(const LikelihoodField& /*field*/) {
  const int N = static_cast<int>(particles_.size());
  std::vector<double> weights(N);
  for (int i = 0; i < N; ++i) {
    weights[i] = std::exp(particles_[i].log_weight);
  }

  // Build CDF.
  std::vector<double> cdf(N);
  cdf[0] = weights[0];
  for (int i = 1; i < N; ++i) {
    cdf[i] = cdf[i - 1] + weights[i];
  }

  // Systematic resampling.
  std::uniform_real_distribution<double> u01(0.0, 1.0 / N);
  double u = u01(rng_);

  std::vector<Particle> new_particles(N);
  int j = 0;
  for (int i = 0; i < N; ++i) {
    while (j < N - 1 && u > cdf[j]) ++j;
    new_particles[i] = particles_[j];
    u += 1.0 / N;
  }
  particles_ = std::move(new_particles);
}

void ParticleFilter::add_roughening() {
  std::normal_distribution<float> pos_noise(0.0f, pf_params_.roughening_pos_m);
  std::normal_distribution<float> ang_noise(0.0f, pf_params_.roughening_angle_rad);
  for (auto& p : particles_) {
    p.x     += pos_noise(rng_);
    p.y     += pos_noise(rng_);
    p.theta  = normalize_angle(p.theta + ang_noise(rng_));
  }
}

std::tuple<float, float, float> ParticleFilter::weighted_mean_pose() const {
  double wx = 0.0, wy = 0.0;
  double sin_sum = 0.0, cos_sum = 0.0;

  for (const auto& p : particles_) {
    double w = std::exp(p.log_weight);
    wx       += w * p.x;
    wy       += w * p.y;
    sin_sum  += w * std::sin(p.theta);
    cos_sum  += w * std::cos(p.theta);
  }

  return {static_cast<float>(wx), static_cast<float>(wy),
          static_cast<float>(std::atan2(sin_sum, cos_sum))};
}

void ParticleFilter::build_kld_table() {
  const double z = 2.326;  // z_{1-0.01}
  const double eps = pf_params_.kld_epsilon;
  kld_table_.resize(pf_params_.kld_max_particles + 1, pf_params_.kld_min_particles);
  for (int k = 2; k <= pf_params_.kld_max_particles; ++k) {
    double km1 = k - 1.0;
    double inner = 1.0 - 2.0/(9.0*km1) + std::sqrt(2.0/(9.0*km1)) * z;
    int n = static_cast<int>(std::ceil(km1 / (2.0*eps) * inner*inner*inner));
    kld_table_[k] = std::clamp(n, pf_params_.kld_min_particles,
                                   pf_params_.kld_max_particles);
  }
}

void ParticleFilter::resample_kld(const LikelihoodField& /*field*/) {
  const int N = static_cast<int>(particles_.size());
  // Build CDF
  std::vector<double> cdf(N);
  cdf[0] = std::exp(particles_[0].log_weight);
  for (int i = 1; i < N; ++i)
    cdf[i] = cdf[i-1] + std::exp(particles_[i].log_weight);

  const float inv_pos = 1.0f / pf_params_.kld_bin_size_m;
  const float inv_ang = 1.0f / pf_params_.kld_bin_size_rad;

  // Bin hash: pack three 21-bit signed-offset integers into uint64_t
  constexpr int32_t OFF = 1 << 20;
  auto bin_key = [&](float x, float y, float th) -> uint64_t {
    int32_t ix = static_cast<int32_t>(std::floor(x  * inv_pos)) + OFF;
    int32_t iy = static_cast<int32_t>(std::floor(y  * inv_pos)) + OFF;
    int32_t it = static_cast<int32_t>(std::floor(th * inv_ang)) + OFF;
    return (uint64_t(ix & 0x1FFFFF) << 42) |
           (uint64_t(iy & 0x1FFFFF) << 21) |
           (uint64_t(it & 0x1FFFFF));
  };

  std::unordered_set<uint64_t> bins;
  bins.reserve(512);
  std::vector<Particle> out;
  out.reserve(pf_params_.kld_max_particles);

  std::uniform_real_distribution<double> u01(0.0, 1.0/N);
  double u = u01(rng_);
  int j = 0, k = 0, target = pf_params_.kld_min_particles;

  for (int drawn = 0; drawn < pf_params_.kld_max_particles; ++drawn) {
    while (j < N-1 && u > cdf[j]) ++j;
    out.push_back(particles_[j]);
    u += 1.0/N;

    if (bins.insert(bin_key(out.back().x, out.back().y, out.back().theta)).second) {
      ++k;
      target = (k < static_cast<int>(kld_table_.size())) ? kld_table_[k]
                                                          : pf_params_.kld_max_particles;
    }
    if (drawn + 1 >= target) break;
  }
  particles_ = std::move(out);
}

double ParticleFilter::normalize_log_weights() {
  // log-sum-exp trick.
  double log_max = -std::numeric_limits<double>::max();
  for (const auto& p : particles_) {
    log_max = std::max(log_max, p.log_weight);
  }

  double sum = 0.0;
  for (const auto& p : particles_) {
    sum += std::exp(p.log_weight - log_max);
  }

  double log_norm = log_max + std::log(sum);
  for (auto& p : particles_) {
    p.log_weight -= log_norm;
  }
  return log_norm;
}

}  // namespace adapt_mcl
