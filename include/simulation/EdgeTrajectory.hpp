#include "simulation/SimulationInstance.hpp"
#include "simulation/SpeedTargets.hpp"

#include <vector>

namespace cda_rail {

struct InitialEdgeState {
  /**
   * Initial train state on new edge
   */
  ulong  timestep; // [1, n_timesteps]
  size_t edge;
  double position;
  bool   orientation;
  double speed;

  InitialEdgeState(ulong timestep, ulong edge, double position,
                   bool orientation, double speed)
      : timestep(timestep), edge(edge), position(position),
        orientation(orientation), speed(speed) {};
};

class EdgeTrajectory {
  /**
   * Continuous train state on one edge
   * Includes one additional simulation timestep after leaving edge
   */
  ulong  initial_timestep; // [1, n_timesteps]
  size_t edge;
  bool   orientation;

  std::vector<double> position;
  std::vector<double> speed;

public:
  // Simulate edge movement from initial state and v targets
  EdgeTrajectory(const SimulationInstance& instance, const Train& train,
                 InitialEdgeState initial_state, SpeedTargets& v_targets);

  // Return possible next initial edge state
  std::optional<InitialEdgeState>
  get_next_edge(const SimulationInstance& instance, double switch_direction);
};

}; // namespace cda_rail
