#pragma once
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <vector>

namespace cda_rail {

using index_vector = std::vector<size_t>;
using index_set    = std::unordered_set<size_t>;

// Constants
constexpr double INF     = std::numeric_limits<double>::max() / 3;
constexpr double EPS     = 10 * std::numeric_limits<double>::epsilon();
constexpr double GRB_EPS = 1e-4;
constexpr double V_MIN   = 0.3;
constexpr double ROUNDING_PRECISION       = 1;
constexpr double STOP_TOLERANCE           = 10;
constexpr double ABS_PWL_ERROR            = 10;
constexpr double LINE_SPEED_ACCURACY      = 0.1;
constexpr double LINE_SPEED_TIME_ACCURACY = 0.1;
constexpr double MIN_NON_ZERO             = 1.0;

// TODO: Sensible to move any of these

enum class VertexType : std::uint8_t {
  NoBorder    = 0,
  VSS         = 1,
  TTD         = 2,
  NoBorderVSS = 3
};
enum class SolutionStatus : std::uint8_t {
  Optimal    = 0,
  Feasible   = 1,
  Infeasible = 2,
  Timeout    = 3,
  Unknown    = 4
};
enum class ExportOption : std::uint8_t {
  NoExport                        = 0,
  ExportSolution                  = 1,
  ExportSolutionWithInstance      = 2,
  ExportLP                        = 3,
  ExportSolutionAndLP             = 4,
  ExportSolutionWithInstanceAndLP = 5
};
enum class OptimalityStrategy : std::uint8_t {
  Optimal  = 0,
  TradeOff = 1,
  Feasible = 2
};
enum class VelocityRefinementStrategy : std::uint8_t {
  None       = 0,
  MinOneStep = 1
};

} // namespace cda_rail
