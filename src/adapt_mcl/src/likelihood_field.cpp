#include "adapt_mcl/likelihood_field.hpp"

#include <algorithm>
#include <cmath>
#include <execution>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>

namespace adapt_mcl {

LikelihoodField::LikelihoodField(const MapInfo& info,
                                 const std::vector<int8_t>& data,
                                 float sigma_hit,
                                 float cpd_sigma,
                                 float cpd_w)
    : info_(info), sigma_hit_(sigma_hit), cpd_sigma_(cpd_sigma), cpd_w_(cpd_w) {
  if (static_cast<int>(data.size()) != info.width * info.height) {
    throw std::invalid_argument("LikelihoodField: data size mismatch");
  }
  build_distance_field(data);
}

void LikelihoodField::build_distance_field(const std::vector<int8_t>& data) {
  const int N = info_.width * info_.height;
  std::vector<float> dist(N, std::numeric_limits<float>::max());
  free_mask_.assign(N, false);
  occupied_mask_.assign(N, false);

  // Multi-source BFS from all occupied cells.
  std::queue<int> q;
  for (int i = 0; i < N; ++i) {
    if (data[i] > 50) {
      dist[i] = 0.0f;
      q.push(i);
      occupied_mask_[i] = true;
    } else if (data[i] == 0) {
      free_mask_[i] = true;
    }
  }

  // 8-connected neighbors: (dx, dy, cost)
  constexpr int ndx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  constexpr int ndy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  constexpr float ndc[8] = {1.4142f, 1.0f, 1.4142f, 1.0f, 1.0f, 1.4142f, 1.0f, 1.4142f};

  while (!q.empty()) {
    int idx = q.front();
    q.pop();
    int cx = idx % info_.width;
    int cy = idx / info_.width;

    for (int k = 0; k < 8; ++k) {
      int nx = cx + ndx[k];
      int ny = cy + ndy[k];
      if (!in_bounds(nx, ny)) continue;
      float new_dist = dist[idx] + ndc[k] * info_.resolution;
      int nidx = cell_index(nx, ny);
      if (new_dist < dist[nidx]) {
        dist[nidx] = new_dist;
        q.push(nidx);
      }
    }
  }

  // Store raw distances for CPD scoring (independent of sigma_hit).
  dist_field_ = dist;

  // Precompute Gaussian likelihood from distances.
  likelihood_field_.resize(N);
  const float inv_2sigma2 = 1.0f / (2.0f * sigma_hit_ * sigma_hit_);
  for (int i = 0; i < N; ++i) {
    float d = dist[i];
    likelihood_field_[i] = std::exp(-d * d * inv_2sigma2);
  }

  // Collect free cell indices for uniform sampling.
  free_cell_indices_.clear();
  occupied_cell_count_ = 0;
  for (int i = 0; i < N; ++i) {
    if (free_mask_[i]) {
      free_cell_indices_.push_back(i);
    }
    if (occupied_mask_[i]) {
      ++occupied_cell_count_;
    }
  }

  map_area_ = static_cast<float>(info_.width) * info_.resolution
            * static_cast<float>(info_.height) * info_.resolution;
  free_space_area_ = static_cast<float>(free_cell_indices_.size())
                   * info_.resolution * info_.resolution;

  // Precompute CPD field: (1-w)*exp(-d²/2σ²_cpd) + w  in [w, 1].
  // Formula is in [0,1] so it plugs directly into the EM sensor model.
  // The uniform floor 'w' means changed-map regions score w instead of ~0.
  if (cpd_sigma_ > 0.0f) {
    cpd_field_.resize(N);
    const float inv_2sc2    = 1.0f / (2.0f * cpd_sigma_ * cpd_sigma_);
    const float one_minus_w = 1.0f - cpd_w_;
    for (int i = 0; i < N; ++i) {
      float d = dist_field_[i];
      cpd_field_[i] = one_minus_w * std::exp(-d * d * inv_2sc2) + cpd_w_;
    }
  }
}

float LikelihoodField::get_likelihood(float wx, float wy) const {
  int cx = to_cell_x(wx);
  int cy = to_cell_y(wy);
  cx = std::max(0, std::min(cx, info_.width  - 1));
  cy = std::max(0, std::min(cy, info_.height - 1));
  return likelihood_field_[cell_index(cx, cy)];
}

float LikelihoodField::get_cpd_likelihood(float wx, float wy) const {
  if (cpd_field_.empty()) return get_likelihood(wx, wy);
  int cx = to_cell_x(wx);
  int cy = to_cell_y(wy);
  cx = std::max(0, std::min(cx, info_.width  - 1));
  cy = std::max(0, std::min(cy, info_.height - 1));
  return cpd_field_[cell_index(cx, cy)];
}

bool LikelihoodField::is_free(float wx, float wy) const {
  int cx = to_cell_x(wx);
  int cy = to_cell_y(wy);
  if (!in_bounds(cx, cy)) return false;
  return free_mask_[cell_index(cx, cy)];
}

std::tuple<float, float, float> LikelihoodField::sample_free_pose(
    std::mt19937& rng) const {
  if (free_cell_indices_.empty()) {
    // Fallback: return map center.
    return {info_.origin_x + info_.width * info_.resolution * 0.5f,
            info_.origin_y + info_.height * info_.resolution * 0.5f, 0.0f};
  }
  std::uniform_int_distribution<int> cell_dist(
      0, static_cast<int>(free_cell_indices_.size()) - 1);
  int idx = free_cell_indices_[cell_dist(rng)];
  int cx = idx % info_.width;
  int cy = idx / info_.width;

  // World coordinates at cell center.
  float wx = info_.origin_x + (cx + 0.5f) * info_.resolution;
  float wy = info_.origin_y + (cy + 0.5f) * info_.resolution;

  std::uniform_real_distribution<float> angle_dist(-M_PIf32, M_PIf32);
  return {wx, wy, angle_dist(rng)};
}

std::vector<LikelihoodField::PoseCandidate>
LikelihoodField::scan_match_candidates(
    const std::vector<std::array<float, 2>>& endpoints_bl,
    float pos_step_m,
    int   angle_bins,
    int   top_k,
    int   n_rays) const {
  if (endpoints_bl.empty() || free_cell_indices_.empty()) return {};

  // Build subsampled ray index list.
  const int n_total = static_cast<int>(endpoints_bl.size());
  const int nr = std::min(n_rays, n_total);
  std::vector<int> ray_idx(nr);
  for (int i = 0; i < nr; ++i) {
    ray_idx[i] = static_cast<int>((static_cast<float>(i) / nr) * n_total);
  }

  // Precompute per-ray base_link endpoints for the subsampled set.
  std::vector<std::array<float, 2>> rays(nr);
  for (int i = 0; i < nr; ++i) rays[i] = endpoints_bl[ray_idx[i]];

  // Coarse grid step in cells.
  const int cell_step = std::max(1, static_cast<int>(pos_step_m / info_.resolution));
  const float p_unif_floor = 0.033f;  // floor to avoid -inf

  // Precompute heading angles.
  std::vector<float> thetas(angle_bins);
  for (int a = 0; a < angle_bins; ++a) {
    thetas[a] = -M_PIf32 + (2.0f * M_PIf32 * a) / angle_bins;
  }

  // Enumerate candidate cells.
  const int cx_max = (info_.width  - 1) / cell_step;
  const int cy_max = (info_.height - 1) / cell_step;
  const int n_cells = (cx_max + 1) * (cy_max + 1);
  const int n_candidates = n_cells * angle_bins;

  std::vector<PoseCandidate> candidates(n_candidates);

  // Build index array and evaluate in parallel.
  std::vector<int> cand_idx(n_candidates);
  std::iota(cand_idx.begin(), cand_idx.end(), 0);

  std::for_each(std::execution::par, cand_idx.begin(), cand_idx.end(),
      [&](int idx) {
        int a   = idx % angle_bins;
        int ci  = idx / angle_bins;
        int cx  = (ci % (cx_max + 1)) * cell_step;
        int cy  = (ci / (cx_max + 1)) * cell_step;

        if (!in_bounds(cx, cy) || !free_mask_[cell_index(cx, cy)]) {
          candidates[idx] = {0.0f, 0.0f, 0.0f,
                             -std::numeric_limits<float>::max()};
          return;
        }

        float px = info_.origin_x + (cx + 0.5f) * info_.resolution;
        float py = info_.origin_y + (cy + 0.5f) * info_.resolution;
        float pt = thetas[a];
        float ct = std::cos(pt), st = std::sin(pt);

        float log_score = 0.0f;
        for (const auto& ep : rays) {
          float ex = px + ct * ep[0] - st * ep[1];
          float ey = py + st * ep[0] + ct * ep[1];
          float lik = get_likelihood(ex, ey);
          log_score += std::log(lik + p_unif_floor);
        }
        candidates[idx] = {px, py, pt, log_score};
      });

  // Remove invalid (non-free) candidates and extract top-K.
  // Use partial_sort for efficiency.
  int k = std::min(top_k, n_candidates);
  std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end(),
                    [](const PoseCandidate& a, const PoseCandidate& b) {
                      return a.log_score > b.log_score;
                    });

  // Trim to top-k entries that are valid (log_score > -max).
  const float bad = -std::numeric_limits<float>::max();
  int valid_k = 0;
  for (int i = 0; i < k; ++i) {
    if (candidates[i].log_score > bad) ++valid_k;
    else break;
  }
  candidates.resize(valid_k);
  return candidates;
}

std::tuple<float, float, float> LikelihoodField::refine_pose(
    float px, float py, float pt,
    const std::vector<std::array<float, 2>>& endpoints_bl,
    float alpha, float p_uniform, int iters, float step) const {
  // Gradient ascent on sum of log(alpha * p_hit(ep) + (1-alpha) * p_uniform).
  // Numerical gradients via central differences at delta = half a map cell.
  const float delta = info_.resolution * 0.5f;

  auto eval = [&](float x, float y, float t) -> float {
    float ct = std::cos(t), st = std::sin(t);
    float L = 0.0f;
    for (const auto& ep : endpoints_bl) {
      float ex = x + ct * ep[0] - st * ep[1];
      float ey = y + st * ep[0] + ct * ep[1];
      float lik = get_likelihood(ex, ey);
      float mix = alpha * lik + (1.0f - alpha) * p_uniform;
      L += std::log(std::max(mix, 1e-15f));
    }
    return L;
  };

  for (int i = 0; i < iters; ++i) {
    float gx = (eval(px + delta, py, pt) - eval(px - delta, py, pt)) / (2.0f * delta);
    float gy = (eval(px, py + delta, pt) - eval(px, py - delta, pt)) / (2.0f * delta);
    float gt = (eval(px, py, pt + delta) - eval(px, py, pt - delta)) / (2.0f * delta);
    float gnorm = std::sqrt(gx * gx + gy * gy + gt * gt);
    if (gnorm < 1e-6f) break;
    px += step * gx / gnorm;
    py += step * gy / gnorm;
    pt += step * gt / gnorm;
  }
  return {px, py, pt};
}

float LikelihoodField::raycast(float px, float py,
                               float dir_x, float dir_y,
                               float max_range) const {
  const float res = info_.resolution;
  const float eps = 1e-6f;

  int cx = to_cell_x(px);
  int cy = to_cell_y(py);

  // Per-cell delta: distance traveled per unit step in x or y direction.
  float idx = (std::abs(dir_x) < eps) ? 1e9f : res / std::abs(dir_x);
  float idy = (std::abs(dir_y) < eps) ? 1e9f : res / std::abs(dir_y);

  int step_x = (dir_x > 0) ? 1 : -1;
  int step_y = (dir_y > 0) ? 1 : -1;

  // Distance along ray to first x and y cell boundary.
  float tx = (dir_x > 0) ? (info_.origin_x + (cx + 1) * res - px) / (std::abs(dir_x) + eps)
                          : (px - info_.origin_x - cx * res)       / (std::abs(dir_x) + eps);
  float ty = (dir_y > 0) ? (info_.origin_y + (cy + 1) * res - py) / (std::abs(dir_y) + eps)
                          : (py - info_.origin_y - cy * res)       / (std::abs(dir_y) + eps);

  float dist = 0.0f;
  while (dist < max_range) {
    if (!in_bounds(cx, cy)) break;
    if (occupied_mask_[cell_index(cx, cy)]) return dist;
    if (tx < ty) { dist = tx; tx += idx; cx += step_x; }
    else          { dist = ty; ty += idy; cy += step_y; }
  }
  return max_range;
}

std::vector<LikelihoodField::PoseCandidate> LikelihoodField::scan_match_candidates_cpd(
    const std::vector<std::array<float, 2>>& endpoints_bl,
    float pos_step_m,
    int   angle_bins,
    int   top_k,
    int   n_rays,
    float cpd_sigma,
    float cpd_w) const {
  if (endpoints_bl.empty() || free_cell_indices_.empty()) return {};

  const int n_total = static_cast<int>(endpoints_bl.size());
  const int nr = std::min(n_rays, n_total);
  std::vector<int> ray_idx(nr);
  for (int i = 0; i < nr; ++i)
    ray_idx[i] = static_cast<int>((static_cast<float>(i) / nr) * n_total);

  std::vector<std::array<float, 2>> rays(nr);
  for (int i = 0; i < nr; ++i) rays[i] = endpoints_bl[ray_idx[i]];

  const int cell_step = std::max(1, static_cast<int>(pos_step_m / info_.resolution));

  std::vector<float> thetas(angle_bins);
  for (int a = 0; a < angle_bins; ++a)
    thetas[a] = -M_PIf32 + (2.0f * M_PIf32 * a) / angle_bins;

  const int cx_max = (info_.width  - 1) / cell_step;
  const int cy_max = (info_.height - 1) / cell_step;
  const int n_cells = (cx_max + 1) * (cy_max + 1);
  const int n_candidates = n_cells * angle_bins;

  // Precompute CPD constants.
  const float inv_2s2     = 1.0f / (2.0f * cpd_sigma * cpd_sigma);
  const float gauss_coeff = 1.0f / (2.0f * M_PIf32 * cpd_sigma * cpd_sigma);
  const float one_minus_w = 1.0f - cpd_w;
  const float outlier_p   = cpd_w / map_area_;
  const float inv_M       = (occupied_cell_count_ > 0)
                              ? 1.0f / static_cast<float>(occupied_cell_count_)
                              : 1.0f;

  std::vector<PoseCandidate> candidates(n_candidates);
  std::vector<int> cand_idx(n_candidates);
  std::iota(cand_idx.begin(), cand_idx.end(), 0);

  std::for_each(std::execution::par, cand_idx.begin(), cand_idx.end(),
      [&](int idx) {
        int a   = idx % angle_bins;
        int ci  = idx / angle_bins;
        int cx  = (ci % (cx_max + 1)) * cell_step;
        int cy  = (ci / (cx_max + 1)) * cell_step;

        if (!in_bounds(cx, cy) || !free_mask_[cell_index(cx, cy)]) {
          candidates[idx] = {0.0f, 0.0f, 0.0f, -std::numeric_limits<float>::max()};
          return;
        }

        float px = info_.origin_x + (cx + 0.5f) * info_.resolution;
        float py = info_.origin_y + (cy + 0.5f) * info_.resolution;
        float pt = thetas[a];
        float ct = std::cos(pt), st = std::sin(pt);

        float log_score = 0.0f;
        for (const auto& ep : rays) {
          float ex = px + ct * ep[0] - st * ep[1];
          float ey = py + st * ep[0] + ct * ep[1];
          int ecx = std::max(0, std::min(to_cell_x(ex), info_.width  - 1));
          int ecy = std::max(0, std::min(to_cell_y(ey), info_.height - 1));
          float d = dist_field_[cell_index(ecx, ecy)];
          float p_hit = gauss_coeff * std::exp(-d * d * inv_2s2);
          float p_n   = one_minus_w * inv_M * p_hit + outlier_p;
          log_score  += std::log(std::max(p_n, 1e-15f));
        }
        candidates[idx] = {px, py, pt, log_score / static_cast<float>(nr)};
      });

  int k = std::min(top_k, n_candidates);
  std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end(),
                    [](const PoseCandidate& a, const PoseCandidate& b) {
                      return a.log_score > b.log_score;
                    });

  const float bad = -std::numeric_limits<float>::max();
  int valid_k = 0;
  for (int i = 0; i < k; ++i) {
    if (candidates[i].log_score > bad) ++valid_k;
    else break;
  }
  candidates.resize(valid_k);
  return candidates;
}

std::vector<LikelihoodField::PoseCandidate> LikelihoodField::cpd_rerank(
    std::vector<PoseCandidate> candidates,
    const std::vector<std::array<float, 2>>& endpoints_bl,
    int n_rays, float cpd_sigma, float cpd_w) const {
  if (candidates.empty() || endpoints_bl.empty()) return candidates;

  const int n_total = static_cast<int>(endpoints_bl.size());
  const int nr = std::min(n_rays, n_total);

  std::vector<int> ray_idx(nr);
  for (int i = 0; i < nr; ++i)
    ray_idx[i] = static_cast<int>((static_cast<float>(i) / nr) * n_total);

  const float inv_2s2     = 1.0f / (2.0f * cpd_sigma * cpd_sigma);
  const float gauss_coeff = 1.0f / (2.0f * M_PIf32 * cpd_sigma * cpd_sigma);
  const float one_minus_w = 1.0f - cpd_w;
  const float outlier_p   = cpd_w / map_area_;
  const float inv_M       = (occupied_cell_count_ > 0)
                              ? 1.0f / static_cast<float>(occupied_cell_count_)
                              : 1.0f;

  std::vector<int> ci(candidates.size());
  std::iota(ci.begin(), ci.end(), 0);

  std::for_each(std::execution::par, ci.begin(), ci.end(), [&](int k) {
    auto& c = candidates[k];
    const float ct = std::cos(c.theta);
    const float st = std::sin(c.theta);

    float log_sum = 0.0f;
    for (int ri : ray_idx) {
      const auto& ep = endpoints_bl[ri];
      float ex = c.x + ct * ep[0] - st * ep[1];
      float ey = c.y + st * ep[0] + ct * ep[1];

      int cx = std::max(0, std::min(to_cell_x(ex), info_.width  - 1));
      int cy = std::max(0, std::min(to_cell_y(ey), info_.height - 1));
      float d = dist_field_[cell_index(cx, cy)];

      // CPD GMM: (1-w)/M * gauss(d; sigma) + w/area
      float p_hit = gauss_coeff * std::exp(-d * d * inv_2s2);
      float p_n   = one_minus_w * inv_M * p_hit + outlier_p;
      log_sum += std::log(std::max(p_n, 1e-15f));
    }
    c.log_score = log_sum / static_cast<float>(nr);  // mean log per ray
  });

  std::sort(candidates.begin(), candidates.end(),
            [](const PoseCandidate& a, const PoseCandidate& b) {
              return a.log_score > b.log_score;
            });
  return candidates;
}

std::vector<LikelihoodField::PoseCandidate> LikelihoodField::caer_rerank(
    std::vector<PoseCandidate> candidates,
    const std::vector<std::array<float, 2>>& endpoints_bl,
    int   n_rays,
    float inlier_threshold) const {
  if (candidates.empty() || endpoints_bl.empty()) return candidates;

  const int n_total = static_cast<int>(endpoints_bl.size());
  const int nr = std::min(n_rays, n_total);

  // Uniform subsampling indices.
  std::vector<int> ray_idx(nr);
  for (int i = 0; i < nr; ++i)
    ray_idx[i] = static_cast<int>((static_cast<float>(i) / nr) * n_total);

  // Parallel scoring: compute inlier fraction for each candidate.
  std::vector<int> ci(candidates.size());
  std::iota(ci.begin(), ci.end(), 0);

  std::for_each(std::execution::par, ci.begin(), ci.end(), [&](int k) {
    auto& c = candidates[k];
    const float ct = std::cos(c.theta);
    const float st = std::sin(c.theta);
    int inliers = 0;
    int valid   = 0;
    for (int ri : ray_idx) {
      const auto& ep = endpoints_bl[ri];
      float r_actual = std::sqrt(ep[0] * ep[0] + ep[1] * ep[1]);
      if (r_actual < 0.01f) continue;
      ++valid;
      // Rotate unit direction from base_link to world frame.
      float ux    = ep[0] / r_actual;
      float uy    = ep[1] / r_actual;
      float dir_x = ct * ux - st * uy;
      float dir_y = st * ux + ct * uy;
      float r_expected = raycast(c.x, c.y, dir_x, dir_y, r_actual * 1.5f + 1.0f);
      if (std::abs(r_expected - r_actual) < inlier_threshold) ++inliers;
    }
    c.caer_score = (valid > 0) ? static_cast<float>(inliers) / valid : 0.0f;
  });

  // Sort descending by inlier fraction (higher = better).
  std::sort(candidates.begin(), candidates.end(),
            [](const PoseCandidate& a, const PoseCandidate& b) {
              return a.caer_score > b.caer_score;
            });
  return candidates;
}

int LikelihoodField::to_cell_x(float wx) const {
  return static_cast<int>((wx - info_.origin_x) / info_.resolution);
}

int LikelihoodField::to_cell_y(float wy) const {
  return static_cast<int>((wy - info_.origin_y) / info_.resolution);
}

bool LikelihoodField::in_bounds(int cx, int cy) const {
  return cx >= 0 && cx < info_.width && cy >= 0 && cy < info_.height;
}

}  // namespace adapt_mcl
