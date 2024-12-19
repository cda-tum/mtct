#include "simulation/TrainTrajectory.hpp"

cda_rail::TrainTrajectory::TrainTrajectory(const SimulationInstance& instance,
                                           const Train&              train,
                                           RoutingSolution           solution,
                                           InitialEdgeState initial_state) {
  edges.reserve(instance.n_timesteps);
  positions.reserve(instance.n_timesteps);
  orientations.reserve(instance.n_timesteps);
  speeds.reserve(instance.n_timesteps);

  // TODO: generate EdgeTrajectories, concat to TrainTrajectory
}
