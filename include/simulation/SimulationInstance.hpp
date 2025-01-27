#pragma once

#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"

#include <cfloat>

namespace cda_rail::sim {

struct SimulationInstance {
  /**
   * Environment for Microscopic Simulation
   */

  const Network                          network;
  const Timetable                        timetable;
  const std::vector<std::vector<double>> shortest_paths;

  const u_int64_t n_timesteps;
  const u_int64_t n_switch_vars;
  const bool      bidirectional_travel;

public:
  SimulationInstance() = delete;
  // TODO: remove double edges from template networks
  SimulationInstance(Network network, Timetable timetable,
                     bool bidirectional_travel);

  double    get_max_train_speed() const;
  double    get_shortest_track() const;
  u_int64_t get_last_train_departure() const;
};

}; // namespace cda_rail::sim
