#pragma once

namespace adapt_mcl {

struct Particle {
  float x{0.0f};
  float y{0.0f};
  float theta{0.0f};
  double log_weight{0.0};
  float alpha{0.9f};  // inlier fraction from last EM step (for diagnostics)
  float gamma{0.0f};  // short-reading fraction from last EM step (diagnostics)
};

}  // namespace adapt_mcl
