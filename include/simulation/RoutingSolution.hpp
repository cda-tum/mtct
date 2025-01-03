#pragma once

#include "simulation/SimulationInstance.hpp"
#include "simulation/SpeedTargets.hpp"

#include <algorithm>
#include <random>
#include <vector>

namespace cda_rail::sim {

struct RoutingSolution {
  /**
   * Heuristic routing decision variables for a single train
   *
   * @param v_targets speed targets to accelerate towards
   * (timestep [0, n_timesteps - 1], speed [-max_speed,+max_speed])
   * @param switch_directions directions to take at vertices
   * direction [0,1]
   * switch_direction selects from list of available next edges
   */
  SpeedTargets        v_targets;
  std::vector<double> switch_directions;

public:
  RoutingSolution() = delete;
  // Constructors
  // Generate a random solution
  RoutingSolution(const SimulationInstance& instance, const Train& train,
                  std::ranlux24_base& rng_engine);
};

}; // namespace cda_rail::sim
