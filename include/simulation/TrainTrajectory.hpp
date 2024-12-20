#include "simulation/EdgeTrajectory.hpp"

#include <optional>
#include <vector>

namespace cda_rail {

class TrainTrajectory {
  /**
   * Path of a single train over entire timespan
   */
  std::vector<EdgeTrajectory>   edge_trajectories;
  std::vector<InitialEdgeState> initial_edge_states;

public:
  TrainTrajectory(const SimulationInstance& instance, const Train& train,
                  RoutingSolution solution, InitialEdgeState init_state);
};

}; // namespace cda_rail
