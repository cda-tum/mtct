#include "simulation/SimulationInstance.hpp"

#include <random>

namespace cda_rail {

class RoutingSolver {
  /**
   * Performs heuristic routing in a SimulationInstance
   *
   * @param instance contains constant parameters
   * @param rng_engine used for solution generation
   */

  const SimulationInstance instance;
  const std::ranlux24_base rng_engine;

  RoutingSolver(SimulationInstance instance);
};

}; // namespace cda_rail
