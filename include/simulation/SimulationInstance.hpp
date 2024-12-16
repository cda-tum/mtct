#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"

#include <cfloat>
#include <stdint.h>

namespace cda_rail {

struct SimulationInstance {
  /**
   * Environment for Microscopic Simulation
   */

  const Network   network;
  const Timetable timetable;

  const uint n_timesteps;
  const uint n_v_target_vars;
  const uint n_switch_vars;

  SimulationInstance(Network network, Timetable timetable, uint n_timesteps,
                     uint n_v_target_vars)
      : network(network), timetable(timetable), n_timesteps(n_timesteps),
        n_v_target_vars(n_v_target_vars),
        n_switch_vars(std::ceil((get_max_train_speed() * n_timesteps) /
                                get_shortest_track())) {};

  double get_max_train_speed() const;
  double get_shortest_track() const;
};

}; // namespace cda_rail
