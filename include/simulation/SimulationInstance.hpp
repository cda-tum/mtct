#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"

#include <cfloat>

namespace cda_rail {

struct SimulationInstance {
  /**
   * Environment for Microscopic Simulation
   */

  const Network   network;
  const Timetable timetable;

  const u_int64_t n_timesteps;
  const u_int64_t n_v_target_vars;
  const u_int64_t n_switch_vars;

public:
  SimulationInstance() = delete;
  SimulationInstance(Network network, Timetable timetable,
                     u_int64_t n_v_target_vars)
      : network(network), timetable(timetable),
        n_timesteps(get_last_train_departure()),
        n_v_target_vars(n_v_target_vars),
        n_switch_vars(std::ceil((get_max_train_speed() * n_timesteps) /
                                get_shortest_track())) {};

  double    get_max_train_speed() const;
  double    get_shortest_track() const;
  u_int64_t get_last_train_departure() const;
};

}; // namespace cda_rail
