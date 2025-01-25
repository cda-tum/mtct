
#include "simulation/SolverResult.hpp"

cda_rail::sim::SolverResult::SolverResult(const SimulationInstance& instance)
    : solutions(RoutingSolutionSet{}),
      trajectories(TrainTrajectorySet(instance)), collision_score(0) {}

cda_rail::sim::SolverResult::SolverResult(const SimulationInstance& instance,
                                          const RoutingSolutionSet& solutions)
    : solutions(solutions),
      trajectories(TrainTrajectorySet(instance, solutions)) {
  if (solutions.solutions.size() != trajectories.size())
    throw std::invalid_argument(
        "Solutions and Trajectories are not the same size");

  for (auto const& [train_name, traj] : trajectories.get_map()) {
    stop_scores.insert_or_assign(train_name, stop_penalty(traj));
  }

  for (auto const& [train_name, traj] : trajectories.get_map()) {
    destination_scores.insert_or_assign(train_name, destination_penalty(traj));
  }

  collision_score = collision_penalty(trajectories);
}

void cda_rail::sim::SolverResult::insert_or_assign(
    const RoutingSolution& solution, const TrainTrajectory& trajectory) {
  solutions.solutions.insert_or_assign(trajectory.get_train().name, solution);
  trajectories.insert_or_assign(trajectory.get_train().name, trajectory);

  stop_scores.insert_or_assign(trajectory.get_train().name,
                               stop_penalty(trajectory));
  destination_scores.insert_or_assign(trajectory.get_train().name,
                                      destination_penalty(trajectory));

  collision_score = collision_penalty(trajectories);
}

const cda_rail::sim::RoutingSolutionSet&
cda_rail::sim::SolverResult::get_solutions() const {
  return solutions;
}

const cda_rail::sim::TrainTrajectorySet&
cda_rail::sim::SolverResult::get_trajectories() const {
  return trajectories;
}

double cda_rail::sim::SolverResult::get_score() const {
  double score_sum = 0;

  for (auto const& [train_name, score] : stop_scores) {
    score_sum = score_sum + score;
  }

  for (auto const& [train_name, score] : destination_scores) {
    score_sum = score_sum + score;
  }

  score_sum = score_sum + collision_score;

  return score_sum;
}
