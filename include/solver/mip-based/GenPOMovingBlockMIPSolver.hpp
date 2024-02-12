#include "datastructure/GeneralTimetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

namespace cda_rail::solver::mip_based {
template <typename T>
class GenPOMovingBlockMIPSolver
    : public GeneralMIPSolver<
          instances::GeneralPerformanceOptimizationInstance<T>,
          instances::SolGeneralPerformanceOptimizationInstance<T>> {
  static_assert(std::is_base_of_v<BaseGeneralSchedule, T>,
                "T must be a child of BaseGeneralSchedule");
  static_assert(instances::HasTimeType<T>::value,
                "T must have a time_type() method");

public:
  GenPOMovingBlockMIPSolver() = default;

  explicit GenPOMovingBlockMIPSolver(
      const instances::GeneralPerformanceOptimizationInstance<T>& instance)
      : GeneralMIPSolver<
            instances::GeneralPerformanceOptimizationInstance<T>,
            instances::SolGeneralPerformanceOptimizationInstance<T>>(
            instance){};

  explicit GenPOMovingBlockMIPSolver(const std::filesystem::path& p)
      : GeneralMIPSolver<
            instances::GeneralPerformanceOptimizationInstance<T>,
            instances::SolGeneralPerformanceOptimizationInstance<T>>(p){};

  explicit GenPOMovingBlockMIPSolver(const std::string& path)
      : GeneralMIPSolver<
            instances::GeneralPerformanceOptimizationInstance<T>,
            instances::SolGeneralPerformanceOptimizationInstance<T>>(path){};

  explicit GenPOMovingBlockMIPSolver(const char* path)
      : GeneralMIPSolver<
            instances::GeneralPerformanceOptimizationInstance<T>,
            instances::SolGeneralPerformanceOptimizationInstance<T>>(path){};

  ~GenPOMovingBlockMIPSolver() = default;
};
} // namespace cda_rail::solver::mip_based
