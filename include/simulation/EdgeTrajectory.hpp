#include "simulation/RoutingSolution.hpp"
#include "simulation/SimulationInstance.hpp"

#include <optional>
#include <vector>

namespace cda_rail {

enum EdgeTransitionOutcome {
  NORMAL,
  OVERSPEED,
  DEADEND,
  PLANNED_STOP,
  TIME_END,
};

struct TrainState {
  /**
   * Single train state
   */
  ulong  timestep;    // [1, n_timesteps]
  size_t edge;        // [0, network.edges.size() - 1]
  double position;    // [0, 1]
  bool   orientation; // true, false = forward, backward
  double speed;       // (-Inf, Inf)
};

struct EdgeTransition {
  EdgeTransitionOutcome     outcome;
  std::optional<TrainState> new_state;
};

class EdgeTrajectory {
  /**
   * Continuous train state on one edge
   * Includes one additional simulation timestep after leaving edge
   */
  SimulationInstance& instance;
  Train&              train;

  ulong  initial_timestep; // [1, n_timesteps]
  size_t edge;             // [0, network.edges.size() - 1]
  bool   orientation;      // true, false = forward, backward

  std::vector<double> positions; // [0, 1]
  std::vector<double> speeds;    // (-Inf, Inf)

public:
  // Simulate movement on edge from initial state and v_targets
  EdgeTrajectory(SimulationInstance& instance, Train& train,
                 SpeedTargets& v_targets, TrainState initial_state);

  EdgeTransition get_transition(double switch_direction);

  bool is_planned_stop();

  ulong               get_initial_timestep();
  ulong               get_last_timestep();
  ulong               get_edge();
  bool                get_orientation();
  std::vector<double> get_positions();
  std::vector<double> get_speeds();
};

}; // namespace cda_rail
