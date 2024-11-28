#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"

namespace cda_rail {

struct SimulationParameters {
  u_int64_t n_timesteps;
  u_int64_t n_v_target_vars;
};

class SimulationInstance {
  /**
   * Environment for Microscopic Simulation
   */

  const Network              network;
  const Timetable            timetable;
  const SimulationParameters parameters;

public:
  SimulationInstance(Network network, Timetable timetable,
                     SimulationParameters parameters)
      : network(network), timetable(timetable), parameters(parameters) {};
};

}; // namespace cda_rail
