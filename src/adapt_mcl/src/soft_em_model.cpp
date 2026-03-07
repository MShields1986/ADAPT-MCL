#include "adapt_mcl/soft_em_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <execution>
#include <utility>
#include <vector>

namespace adapt_mcl {

SoftEmModel::SoftEmModel(const SoftEmParams& params) : params_(params) {}

void SoftEmModel::compute_weights(
    std::vector<Particle>& particles,
    const std::vector<std::array<float, 2>>& endpoints_bl,
    const LikelihoodField& field) const {
  const int n_total = static_cast<int>(endpoints_bl.size());
  if (n_total == 0) return;

  // Stratified subsampling of endpoint indices.
  const int n_rays = std::min(params_.n_rays, n_total);
  std::vector<int> idx(n_rays);
  for (int i = 0; i < n_rays; ++i) {
    idx[i] = static_cast<int>((static_cast<float>(i) / n_rays) * n_total);
  }

  const float p_unif = params_.p_uniform;

  const bool use_z_short = params_.use_z_short;

  std::for_each(
      std::execution::par, particles.begin(), particles.end(),
      [&](Particle& p) {
        const float cos_t = std::cos(p.theta);
        const float sin_t = std::sin(p.theta);

        // Likelihoods per subsampled ray — thread-local to avoid per-particle allocation.
        thread_local std::vector<float> liks;
        thread_local std::vector<float> zs;
        if (static_cast<int>(liks.size()) < n_rays) liks.resize(n_rays);
        if (static_cast<int>(zs.size()) < n_rays) zs.resize(n_rays);
        int n_valid = 0;

        for (int i = 0; i < n_rays; ++i) {
          const auto& ep = endpoints_bl[idx[i]];
          // Transform base_link endpoint to map frame.
          float ex = p.x + cos_t * ep[0] - sin_t * ep[1];
          float ey = p.y + sin_t * ep[0] + cos_t * ep[1];
          liks[n_valid] = field.get_likelihood(ex, ey);
          zs[n_valid]   = std::sqrt(ep[0]*ep[0] + ep[1]*ep[1]);
          ++n_valid;
        }

        if (n_valid == 0) {
          p.log_weight += std::log(p_unif) * n_rays;
          return;
        }

        if (use_z_short) {
          auto [alpha, gamma] = em_estimate_alpha_gamma(liks.data(), zs.data(), n_valid);
          p.alpha = alpha;
          p.gamma = gamma;
          const float lambda = params_.lambda_short;
          double log_w = 0.0;
          for (int i = 0; i < n_valid; ++i) {
            float p_short = lambda * std::exp(-lambda * zs[i]);
            float mix = alpha * liks[i] + gamma * p_short + (1.0f-alpha-gamma) * p_unif;
            mix = std::max(mix, 1e-15f);
            log_w += std::log(static_cast<double>(mix));
          }
          p.log_weight += log_w / std::pow(static_cast<double>(n_valid), params_.norm_exponent);
        } else {
          float alpha = em_estimate_alpha(liks.data(), n_valid);
          p.alpha = alpha;

          double log_w = 0.0;
          for (int i = 0; i < n_valid; ++i) {
            float mix = alpha * liks[i] + (1.0f - alpha) * p_unif;
            mix = std::max(mix, 1e-15f);
            log_w += std::log(static_cast<double>(mix));
          }
          p.log_weight += log_w / std::pow(static_cast<double>(n_valid), params_.norm_exponent);
        }
      });
}

float SoftEmModel::em_estimate_alpha(const float* likelihoods, int n) const {
  float alpha = params_.alpha_prior / (params_.alpha_prior + params_.beta_prior);
  const float p_unif = params_.p_uniform;

  for (int iter = 0; iter < params_.em_iters; ++iter) {
    float sum_r = 0.0f;
    for (int i = 0; i < n; ++i) {
      float num   = alpha * likelihoods[i];
      float denom = num + (1.0f - alpha) * p_unif;
      sum_r += (denom > 1e-30f) ? (num / denom) : 0.0f;
    }
    alpha = (sum_r + params_.alpha_prior)
            / (static_cast<float>(n) + params_.alpha_prior + params_.beta_prior);
    alpha = std::max(0.01f, std::min(0.99f, alpha));
  }
  return alpha;
}

std::pair<float,float> SoftEmModel::em_estimate_alpha_gamma(
    const float* likelihoods, const float* ranges, int n) const {
  const float p_unif  = params_.p_uniform;
  const float lambda  = params_.lambda_short;
  const float total_prior = params_.alpha_prior + params_.beta_prior + params_.gamma_prior;
  float alpha = params_.alpha_prior / total_prior;
  float gamma = params_.gamma_prior / total_prior;

  for (int iter = 0; iter < params_.em_iters; ++iter) {
    float sum_r_hit = 0.0f, sum_r_short = 0.0f;
    for (int i = 0; i < n; ++i) {
      float p_short = lambda * std::exp(-lambda * ranges[i]);
      float denom = alpha * likelihoods[i] + gamma * p_short
                    + (1.0f - alpha - gamma) * p_unif;
      if (denom < 1e-30f) continue;
      sum_r_hit   += alpha * likelihoods[i] / denom;
      sum_r_short += gamma * p_short        / denom;
    }
    float denom_m = static_cast<float>(n) + total_prior;
    alpha = std::clamp((sum_r_hit   + params_.alpha_prior) / denom_m, 0.01f, 0.98f);
    gamma = std::clamp((sum_r_short + params_.gamma_prior) / denom_m, 0.01f, 0.98f);
    if (alpha + gamma > 0.99f) {
      float scale = 0.99f / (alpha + gamma);
      alpha *= scale; gamma *= scale;
    }
  }
  return {alpha, gamma};
}

}  // namespace adapt_mcl
