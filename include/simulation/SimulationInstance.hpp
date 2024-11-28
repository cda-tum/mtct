#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"

#include "gtest/gtest_prod.h"
#include <cfloat>

namespace cda_rail {

class SimulationInstance {
  /**
   * Environment for Microscopic Simulation
   */

  Network   network;
  Timetable timetable;

  u_int64_t n_timesteps;
  u_int64_t n_v_target_vars;
  u_int64_t n_switch_timesteps;

public:
  SimulationInstance(Network network, Timetable timetable,
                     u_int64_t n_timesteps, u_int64_t n_v_target_vars)
      : network(network), timetable(timetable), n_timesteps(n_timesteps),
        n_v_target_vars(n_v_target_vars) {
    n_switch_timesteps =
        std::ceil((get_max_train_speed() * n_timesteps) / get_shortest_track());
  };

  double get_max_train_speed();
  double get_shortest_track();
};
}; // namespace cda_rail
