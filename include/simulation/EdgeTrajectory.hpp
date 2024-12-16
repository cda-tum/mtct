#include "simulation/RoutingSolution.hpp"
#include "simulation/SimulationInstance.hpp"

#include <vector>

namespace cda_rail {

class InitialEdgeState {
  /**
   * Initial train state on new edge
   */
  uint   timestep;
  uint   edge;
  double position;
  bool   orientation;
  double speed;

  InitialEdgeState(uint timestep, uint edge, double position, bool orientation,
                   double speed)
      : timestep(timestep), edge(edge), position(position),
        orientation(orientation), speed(speed) {};
};

class EdgeTrajectory {
  /**
   * Continuous train state on one edge
   */
  uint                initial_timestep;
  uint                edge;
  bool                orientation;
  std::vector<double> position;
  std::vector<double> speed;

  // Simulate edge movement from initial state and v targets
  EdgeTrajectory(const SimulationInstance& instance,
                 InitialEdgeState initial_state, const RoutingSolution& sol);

  // Return possible next initial edge state
  std::optional<InitialEdgeState> get_next_edge(double switch_direction);
};

}; // namespace cda_rail
