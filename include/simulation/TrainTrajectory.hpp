#include "simulation/EdgeTrajectory.hpp"
#include "simulation/RoutingSolution.hpp"

#include <vector>

namespace cda_rail {

class TrainTrajectory {
  /**
   * Path of a single train over entire timespan
   */
  std::vector<size_t> edges;
  std::vector<double> positions;
  std::vector<bool>   orientations;
  std::vector<double> speeds;

public:
  TrainTrajectory(const SimulationInstance& instance, const Train& train,
                  RoutingSolution solution, InitialEdgeState initial_state);
};

}; // namespace cda_rail
