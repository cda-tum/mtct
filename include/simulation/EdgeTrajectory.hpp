#include "simulation/RoutingSolution.hpp"
#include "simulation/SimulationInstance.hpp"

#include <optional>
#include <vector>

namespace cda_rail {

struct TrainState {
  /**
   * Single train state
   */
  ulong  timestep;    // [0, n_timesteps - 1]
  size_t edge;        // [0, network.edges.size() - 1]
  double position;    // [0, 1]
  bool   orientation; // true, false = forward, backward
  double speed;       // (-Inf, Inf)
};

enum EdgeEntryOutcome {
  NORMAL,
  OVERSPEED,
  DEADEND,
  PLANNED_STOP,
  TIME_END,
};

struct EdgeEntry {
  EdgeEntryOutcome          outcome;
  std::optional<TrainState> new_state;
};

struct EdgeExit {
  bool   exit_point;          // true, false = forward, backward
  size_t traversed_node;      // [0, network.edges.size() - 1]
  bool   traversal_direction; // true, false = forward, backward
  double leftover_movement;   // (0, Inf)
  double traversal_speed;     // (-Inf, Inf)
};

class EdgeTrajectory {
  /**
   * Continuous train state on one edge
   */
  SimulationInstance& instance;
  Train&              train;

  ulong  initial_timestep; // [0, n_timesteps - 1]
  size_t edge;             // [0, network.edges.size() - 1]
  bool   orientation;      // true, false = forward, backward

  std::vector<double> positions; // [0, 1]
  std::vector<double> speeds;    // (-Inf, Inf)

  std::optional<EdgeExit> transition;

private:
  EdgeExit determine_exit(double exit_position, double exit_speed) const;

public:
  // Simulate movement on edge from initial state and v_targets
  EdgeTrajectory(SimulationInstance& instance, Train& train,
                 SpeedTargets& v_targets, TrainState initial_state);

  EdgeEntry enter_next_edge(double switch_direction) const;

  bool is_planned_stop() const;

  ulong                          get_initial_timestep() const;
  ulong                          get_last_timestep() const;
  ulong                          get_edge() const;
  bool                           get_orientation() const;
  const std::vector<double>&     get_positions() const;
  const std::vector<double>&     get_speeds() const;
  const std::optional<EdgeExit>& get_transition() const;
};

}; // namespace cda_rail
