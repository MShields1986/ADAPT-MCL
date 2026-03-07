#include "adapt_mcl/likelihood_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace adapt_mcl {

LikelihoodField::LikelihoodField(const MapInfo& info,
                                 const std::vector<int8_t>& data,
                                 float sigma_hit)
    : info_(info), sigma_hit_(sigma_hit) {
  if (static_cast<int>(data.size()) != info.width * info.height) {
    throw std::invalid_argument("LikelihoodField: data size mismatch");
  }
  build_distance_field(data);
}

void LikelihoodField::build_distance_field(const std::vector<int8_t>& data) {
  const int N = info_.width * info_.height;
  std::vector<float> dist(N, std::numeric_limits<float>::max());
  free_mask_.assign(N, false);

  // Multi-source BFS from all occupied cells.
  std::queue<int> q;
  for (int i = 0; i < N; ++i) {
    if (data[i] > 50) {
      dist[i] = 0.0f;
      q.push(i);
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

  // Precompute Gaussian likelihood from distances.
  likelihood_field_.resize(N);
  const float inv_2sigma2 = 1.0f / (2.0f * sigma_hit_ * sigma_hit_);
  for (int i = 0; i < N; ++i) {
    float d = dist[i];
    likelihood_field_[i] = std::exp(-d * d * inv_2sigma2);
  }

  // Collect free cell indices for uniform sampling.
  free_cell_indices_.clear();
  for (int i = 0; i < N; ++i) {
    if (free_mask_[i]) {
      free_cell_indices_.push_back(i);
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
