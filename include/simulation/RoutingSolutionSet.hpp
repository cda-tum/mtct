#pragma once

#include "simulation/RoutingSolution.hpp"
#include "simulation/SimulationInstance.hpp"

namespace cda_rail {

struct RoutingSolutionSet {
  /**
   * Routing solutions for all trains in a timetable
   *
   * @param solutions Map containing train name and corresponding
   * RoutingSolution
   */

  std::unordered_map<std::string, RoutingSolution> solutions;

public:
  RoutingSolutionSet() = delete;
  // Generate random solutions
  RoutingSolutionSet(const SimulationInstance& instance,
                     std::ranlux24_base&       rng_engine);
};

}; // namespace cda_rail
