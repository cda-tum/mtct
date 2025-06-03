#pragma once

#include "simulation/RoutingSolution.hpp"
#include "simulation/SimulationInstance.hpp"

#include <algorithm>

namespace cda_rail::sim {

struct RoutingSolutionSet {
  /**
   * Routing solutions for all trains in a timetable
   *
   * @param solutions Map containing train name and corresponding
   * RoutingSolution
   */

  std::unordered_map<std::string, RoutingSolution> solutions;

public:
  // No Solutions
  RoutingSolutionSet();
  // Stationary solutions
  RoutingSolutionSet(const SimulationInstance& instance);
  // Random solutions
  RoutingSolutionSet(const SimulationInstance& instance,
                     std::ranlux24_base&       rng_engine);
  RoutingSolutionSet(const SimulationInstance&          instance,
                     const std::function<double(void)>& rnd01);

  void perturb(const SimulationInstance& instance, double fraction,
               std::ranlux24_base& rng_engine);
  void perturb(const SimulationInstance& instance, double fraction,
               const std::function<double(void)>& rnd01);
};

}; // namespace cda_rail::sim
