#pragma once

#include "probleminstances/GeneralProblemInstance.hpp"

#include <type_traits>

namespace cda_rail::solver {
template <typename T, typename S> class GeneralSolver {
  static_assert(
      std::is_base_of_v<cda_rail::instances::GeneralProblemInstance, T>,
      "T must be a child of GeneralProblemInstance");
  static_assert(
      std::is_base_of_v<cda_rail::instances::SolGeneralProblemInstance<T>, S>,
      "S must be a child of SolGeneralProblemInstance<T>");

protected:
  T instance;

  GeneralSolver() = default;
  explicit GeneralSolver(T instance) : instance(std::move(instance)){};

public:
  [[nodiscard]] const T& get_instance() const { return instance; }
  [[nodiscard]] T&       editable_instance() { return instance; }

  [[nodiscard]] virtual S solve() = 0;

  virtual ~GeneralSolver() = default;
};
} // namespace cda_rail::solver
