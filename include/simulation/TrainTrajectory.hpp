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
                  RoutingSolution solution);
};

InitialEdgeState
init_train_state_from_schedule(const SimulationInstance& instance,
                               const Train&              train);

}; // namespace cda_rail
