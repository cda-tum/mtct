#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <vector>

namespace cda_rail {

using index_vector = std::vector<size_t>;
using index_set    = std::unordered_set<size_t>;

// Constants
constexpr double INF     = std::numeric_limits<double>::max() / 3;
constexpr double EPS     = 1000 * std::numeric_limits<double>::epsilon();
constexpr double GRB_EPS = 1e-4;
constexpr double V_MIN   = 0.3;
constexpr double ROUNDING_PRECISION       = 1;
constexpr double STOP_TOLERANCE           = 10;
constexpr double ABS_PWL_ERROR            = 10;
constexpr double LINE_SPEED_ACCURACY      = 0.1;
constexpr double LINE_SPEED_TIME_ACCURACY = 0.1;
constexpr double MIN_NON_ZERO             = 0.1;
constexpr double MIN_OCCUPIED_LENGTH      = 1;

// TODO: Sensible to move any of these

/**
 * @brief Which kind of block border a vertex is.
 *
 * - `NoBorder`: no border at all, the incident edges belong to the same
 *   unbreakable (TTD) section.
 * - `VSS`: border of a virtual subsection.
 * - `TTD`: border of a train detection section.
 * - `NoBorderVSS`: no border in the given network, but a possible position of
 *   a VSS border, which is where `discretize` splits an edge.
 */
enum class VertexType : std::uint8_t {
  NoBorder    = 0,
  VSS         = 1,
  TTD         = 2,
  NoBorderVSS = 3
};
/** @brief What a solver was able to say about the instance it was given. */
enum class SolutionStatus : std::uint8_t {
  Optimal    = 0,
  Feasible   = 1,
  Infeasible = 2,
  Timeout    = 3,
  Unknown    = 4
};
/**
 * @brief How hard a solver tries to prove optimality.
 *
 * `Optimal` solves to optimality, `TradeOff` stops at the first solution
 * found, and `Feasible` additionally asks the solver itself to focus on
 * finding one.
 */
enum class OptimalityStrategy : std::uint8_t {
  Optimal  = 0,
  TradeOff = 1,
  Feasible = 2
};
/**
 * @brief How the discrete velocities of a vertex are chosen.
 *
 * `None` uses an equidistant grid of the desired accuracy. `MinOneStep`
 * refines it further where needed, so that two consecutive velocities can be
 * reached from one another on the shortest neighboring edge.
 */
enum class VelocityRefinementStrategy : std::uint8_t {
  None       = 0,
  MinOneStep = 1
};

} // namespace cda_rail
