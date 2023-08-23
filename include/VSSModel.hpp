#pragma once
#include "CustomExceptions.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

namespace cda_rail::vss {
using SeparationFunction = std::function<double(size_t, size_t)>;

enum class ModelType {
  DISCRETE     = 0,
  CONTINUOUS   = 1,
  INFERRED     = 2,
  INFERRED_ALT = 3
};

namespace functions {
[[nodiscard]] static double uniform(size_t i, size_t n) {
  double ret_val = (static_cast<double>(i) + 1) / static_cast<double>(n);
  if (ret_val > 1) {
    ret_val = 1;
  }
  return ret_val;
}

[[nodiscard]] static size_t max_n_blocks(const SeparationFunction& sep_func,
                                         double                    min_frac) {
  if (min_frac < 0 || min_frac > 1) {
    throw std::invalid_argument("min_frac must be in [0, 1].");
  }

  for (size_t n = 2; static_cast<double>(n) <= 1 / min_frac; ++n) {
    if (sep_func(0, n) < min_frac || 1 - sep_func(n - 2, n) < min_frac) {
      return n - 1;
    }
    for (size_t i = 1; i < n - 1; ++i) {
      if (sep_func(i, n) - sep_func(i - 1, n) < min_frac) {
        return n - 1;
      }
    }
  }

  return static_cast<size_t>(std::floor(1 / min_frac));
}
} // namespace functions

class Model {
private:
  ModelType                       model_type;
  std::vector<SeparationFunction> separation_functions = {};

  [[nodiscard]] bool check_consistency() const {
    // The following must hold
    // DISCRETE -> 1 separation function
    if (model_type == ModelType::DISCRETE && separation_functions.size() == 1) {
      return true;
    }
    // CONTINUOUS -> no further information
    if (model_type == ModelType::CONTINUOUS && separation_functions.empty()) {
      return true;
    }
    // PREDEFINED -> >= 1 separation function
    if (model_type == ModelType::INFERRED && !separation_functions.empty()) {
      return true;
    }
    // CUSTOM -> >= 1 separation functions
    if (model_type == ModelType::INFERRED_ALT &&
        !separation_functions.empty()) {
      return true;
    }

    return false;
  }

public:
  // Constructors
  explicit Model(ModelType model_type_input) : model_type(model_type_input) {
    if (!check_consistency()) {
      throw cda_rail::exceptions::ConsistencyException(
          "Model type and separation types/functions are not consistent.");
    }
  }
  explicit Model(ModelType                       model_type_input,
                 std::vector<SeparationFunction> separation_functions_input)
      : model_type(model_type_input),
        separation_functions(std::move(separation_functions_input)) {
    if (!check_consistency()) {
      throw cda_rail::exceptions::ConsistencyException(
          "Model type and separation types/functions are not consistent.");
    }
  }
  // No default constructor
  Model() = delete;

  // Getters
  [[nodiscard]] const ModelType& get_model_type() const { return model_type; }
  [[nodiscard]] const std::vector<SeparationFunction>&
  get_separation_functions() const {
    if (separation_functions.empty()) {
      throw std::logic_error("Model has no separation functions.");
    }
    return separation_functions;
  }
};
} // namespace cda_rail::vss
