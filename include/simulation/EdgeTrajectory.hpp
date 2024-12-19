#include "simulation/SimulationInstance.hpp"
#include "simulation/SpeedTargets.hpp"

#include <vector>

namespace cda_rail {

struct InitialEdgeState {
  /**
   * Initial train state on new edge
   */
  ulong  timestep;    // [1, n_timesteps]
  size_t edge;        // [0, network.edges.size() - 1]
  double position;    // [0, 1]
  bool   orientation; // true, false = forward, backward
  double speed;       // (-Inf, Inf)
};

class EdgeTrajectory {
  /**
   * Continuous train state on one edge
   * Includes one additional simulation timestep after leaving edge
   */
  ulong  initial_timestep; // [1, n_timesteps]
  size_t edge;             // [0, network.edges.size() - 1]
  bool   orientation;      // true, false = forward, backward

  std::vector<double> positions; // [0, 1]
  std::vector<double> speeds;    // (-Inf, Inf)

public:
  // Simulate movement on edge from initial state and v_targets
  EdgeTrajectory(const SimulationInstance& instance, const Train& train,
                 InitialEdgeState initial_state, SpeedTargets& v_targets);

  // Contains edge transition logic
  std::optional<InitialEdgeState>
  get_next_edge(const SimulationInstance& instance, double switch_direction);
};

}; // namespace cda_rail
