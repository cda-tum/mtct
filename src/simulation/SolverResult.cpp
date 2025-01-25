
#include "simulation/SolverResult.hpp"

cda_rail::sim::SolverResult::SolverResult(const SimulationInstance& instance)
    : solutions(RoutingSolutionSet{}),
      trajectories(TrainTrajectorySet(instance)) {}

cda_rail::sim::SolverResult::SolverResult(
    const RoutingSolutionSet& solutions, const TrainTrajectorySet& trajectories)
    : solutions(solutions), trajectories(trajectories) {
  if (solutions.solutions.size() != trajectories.size())
    throw std::invalid_argument(
        "Solutions and Trajectories are not the same size");
}

void cda_rail::sim::SolverResult::insert_or_assign(
    const RoutingSolution& solution, const TrainTrajectory& trajectory) {
  solutions.solutions.insert_or_assign(trajectory.get_train().name, solution);
  trajectories.insert_or_assign(trajectory.get_train().name, trajectory);
}

const cda_rail::sim::RoutingSolutionSet
cda_rail::sim::SolverResult::get_solutions() const {
  return solutions;
}

const cda_rail::sim::TrainTrajectorySet
cda_rail::sim::SolverResult::get_trajectories() const {
  return trajectories;
}
