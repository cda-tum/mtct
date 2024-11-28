#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"

#include <cfloat>
#include <stdint.h>

namespace cda_rail {

struct SimulationInstance {
  /**
   * Environment for Microscopic Simulation
   */

  Network   network;
  Timetable timetable;

  uint64_t n_timesteps;
  uint64_t n_v_target_vars;
  uint64_t n_switch_vars;

  SimulationInstance(Network network, Timetable timetable, uint64_t n_timesteps,
                     uint64_t n_v_target_vars)
      : network(network), timetable(timetable), n_timesteps(n_timesteps),
        n_v_target_vars(n_v_target_vars) {
    n_switch_vars =
        std::ceil((get_max_train_speed() * n_timesteps) / get_shortest_track());
  };

  double get_max_train_speed();
  double get_shortest_track();
};

}; // namespace cda_rail
