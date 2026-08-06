#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace cda_rail::vss {
using SeparationFunction = std::function<double(size_t, size_t)>;

enum class ModelType : std::uint8_t {
  Discrete    = 0,
  Continuous  = 1,
  Inferred    = 2,
  InferredAlt = 3
};

namespace functions {
/**
 * @brief Computes uniformly spaced normalized positions.
 *
 * @param i Zero-based node index.
 * @param n Total number of nodes.
 * @return Normalized position in `[0, 1]`.
 */
[[nodiscard]] static double uniform(size_t i, size_t n) {
  double ret_val = (static_cast<double>(i) + 1) / static_cast<double>(n);
  ret_val        = std::min<double>(ret_val, 1);
  return ret_val;
}

/**
 * @brief Computes a Chebyshev separation value for node positioning.
 *
 * Calculates the normalized position of a node using Chebyshev polynomial
 * spacing, which provides non-uniform distribution with density clustering near
 * boundaries.
 *
 * @param i Node index.
 * @param n Total number of nodes.
 * @return double Normalized position in [0, 1].
 */
[[nodiscard]] static double chebyshev(size_t i, size_t n) {
  if (i >= n - 1) {
    return 1;
  }

  const auto       n_points = static_cast<double>(n) - 1;
  const auto       k        = n_points - static_cast<double>(i);
  constexpr double pi       = std::numbers::pi;
  return 0.5 + (0.5 * std::cos(((2 * k) - 1) * pi / (2 * n_points)));
}

/**
 * @brief Finds the maximum number of blocks with a given minimum separation
 * constraint.
 *
 * @param sep_func Separation function that computes positions for a given block
 * index and total count.
 * @param min_frac Minimum required separation, in the range (0, 1].
 *
 * @return Maximum number of blocks where all separation margins meet the
 * minimum requirement.
 *
 * @throws std::invalid_argument if min_frac is not in (0, 1].
 */
[[nodiscard]] static size_t max_n_blocks(const SeparationFunction& sep_func,
                                         double                    min_frac) {
  constexpr auto eps = 10 * std::numeric_limits<double>::epsilon();

  if (min_frac - eps <= 0 || min_frac > 1 + eps) {
    throw std::invalid_argument("min_frac must be in (0, 1].");
  }

  for (size_t n = 2; static_cast<double>(n) <= (1 / min_frac) + eps; ++n) {
    if (sep_func(0, n) + eps < min_frac ||
        1 - sep_func(n - 2, n) + eps < min_frac) {
      return n - 1;
    }
    for (size_t i = 1; i < n - 1; ++i) {
      if (sep_func(i, n) - sep_func(i - 1, n) + eps < min_frac) {
        return n - 1;
      }
    }
  }

  return static_cast<size_t>(std::floor((1 / min_frac) + eps));
}
} // namespace functions
} // namespace cda_rail::vss
