#include <random>
#include <stdint.h>
#include <vector>

namespace cda_rail {

struct RoutingSolution {
  /**
   * Heuristic routing decision variables for a single train
   *
   * @param v_targets speed targets to accelerate towards
   * (timestep [0,1], speed [0,1])
   * speed 0 is max backward while 1 is max forward
   * @param switch_directions directions to take at vertices
   * direction [0,1]
   * switch_direction selects from list of available next edges
   */
  std::vector<std::tuple<double, double>> v_targets;
  std::vector<double>                     switch_directions;

  // Constructors
  // Generate a random solution
  RoutingSolution(uint64_t n_v_target_vars, uint64_t n_switch_vars,
                  std::ranlux24_base& rng_engine);
  // Alternatively pass in all variables
  RoutingSolution(std::vector<std::tuple<double, double>> v_targets,
                  std::vector<double>                     switch_directions)
      : v_targets(v_targets), switch_directions(switch_directions) {};
};

}; // namespace cda_rail
