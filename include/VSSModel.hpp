#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cda_rail::vss {
using SeparationFunction = std::function<double(size_t, size_t)>;

enum class ModelType : std::uint8_t {
  Discrete    = 0,
  Continuous  = 1,
  Inferred    = 2,
  InferredAlt = 3
};

namespace functions {
[[nodiscard]] static double uniform(size_t i, size_t n) {
  double ret_val = (static_cast<double>(i) + 1) / static_cast<double>(n);
  ret_val        = std::min<double>(ret_val, 1);
  return ret_val;
}

[[nodiscard]] static double chebyshev(size_t i, size_t n) {
  if (i >= n - 1) {
    return 1;
  }

  const auto       n_points = static_cast<double>(n) - 1;
  const auto       k        = n_points - static_cast<double>(i);
  constexpr double pi       = 3.14159265358979323846;
  return 0.5 + (0.5 * std::cos((2 * k - 1) * pi / (2 * n_points)));
}

[[nodiscard]] static size_t max_n_blocks(const SeparationFunction& sep_func,
                                         double                    min_frac) {
  constexpr auto eps = 10 * std::numeric_limits<double>::epsilon();

  if (min_frac - eps <= 0 || min_frac > 1 + eps) {
    throw std::invalid_argument("min_frac must be in (0, 1].");
  }

  for (size_t n = 2; static_cast<double>(n) <= 1 / min_frac + eps; ++n) {
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

class Model {
private:
  ModelType                       model_type       = ModelType::Continuous;
  bool                            only_stop_at_vss = false;
  std::vector<SeparationFunction> separation_functions;

public:
  // Constructors
  explicit Model() = default;
  explicit Model(ModelType model_type_input) : model_type(model_type_input) {}
  explicit Model(ModelType                       model_type_input,
                 std::vector<SeparationFunction> separation_functions_input)
      : model_type(model_type_input),
        separation_functions(std::move(separation_functions_input)) {}
  explicit Model(ModelType                       model_type_input,
                 std::vector<SeparationFunction> separation_functions_input,
                 bool                            only_stop_at_vss_input)
      : model_type(model_type_input),
        separation_functions(std::move(separation_functions_input)),
        only_stop_at_vss(only_stop_at_vss_input) {}

  // Getters
  [[nodiscard]] const ModelType& get_model_type() const { return model_type; }
  [[nodiscard]] bool get_only_stop_at_vss() const { return only_stop_at_vss; }
  [[nodiscard]] const std::vector<SeparationFunction>&
  get_separation_functions() const {
    if (separation_functions.empty()) {
      throw std::logic_error("Model has no separation functions.");
    }
    return separation_functions;
  }

  // Helper
  [[nodiscard]] bool check_consistency() const {
    // The following must hold
    // Discrete -> 1 separation function
    if (model_type == ModelType::Discrete && separation_functions.size() == 1) {
      return true;
    }
    // Continuous -> no further information
    if (model_type == ModelType::Continuous && separation_functions.empty()) {
      return true;
    }
    // PREDEFINED -> >= 1 separation function
    if (model_type == ModelType::Inferred && !separation_functions.empty()) {
      return true;
    }
    // CUSTOM -> >= 1 separation functions
    if (model_type == ModelType::InferredAlt && !separation_functions.empty()) {
      return true;
    }

    return false;
  }
};
} // namespace cda_rail::vss
