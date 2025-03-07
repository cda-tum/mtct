
#include "simulation/SolverResult.hpp"

cda_rail::sim::SolverResult::SolverResult(const SimulationInstance& instance)
    : solutions(RoutingSolutionSet{}),
      trajectories(TrainTrajectorySet(instance)) {}

cda_rail::sim::SolverResult::SolverResult(const SimulationInstance& instance,
                                          const RoutingSolutionSet& solutions)
    : solutions(solutions),
      trajectories(TrainTrajectorySet(instance, solutions)) {
  if (solutions.solutions.size() != trajectories.size())
    throw std::invalid_argument(
        "Solutions and Trajectories are not the same size");

  for (auto const& [train_name, traj] : trajectories.get_map()) {
    double pen = stop_penalty(traj);
    scores.stop_scores.insert_or_assign(train_name, pen);
  }

  for (auto const& [train_name, traj] : trajectories.get_map()) {
    scores.destination_scores.insert_or_assign(train_name,
                                               destination_penalty(traj));
  }

  scores.collision_score = collision_penalty(trajectories);
}

void cda_rail::sim::SolverResult::insert_or_assign(
    const RoutingSolution& solution, const TrainTrajectory& trajectory) {
  solutions.solutions.insert_or_assign(trajectory.get_train().name, solution);
  trajectories.insert_or_assign(trajectory.get_train().name, trajectory);

  scores.stop_scores.insert_or_assign(trajectory.get_train().name,
                                      stop_penalty(trajectory));
  scores.destination_scores.insert_or_assign(trajectory.get_train().name,
                                             destination_penalty(trajectory));

  // TODO: don't need to recompute all pairs for new train
  scores.collision_score = collision_penalty(trajectories);
}

const cda_rail::sim::RoutingSolutionSet&
cda_rail::sim::SolverResult::get_solutions() const {
  return solutions;
}

const cda_rail::sim::TrainTrajectorySet&
cda_rail::sim::SolverResult::get_trajectories() const {
  return trajectories;
}

double cda_rail::sim::ScoreSet::get_score() const {
  double score_sum = 0;

  score_sum += get_collision_score();
  score_sum += get_destination_score();
  score_sum += get_stop_score();

  return score_sum;
}

double cda_rail::sim::ScoreSet::get_collision_score() const {
  return collision_score;
}

double cda_rail::sim::ScoreSet::get_stop_score() const {
  double score_sum = 0;
  for (auto const& [train_name, score] : stop_scores) {
    score_sum = score_sum + score;
  }

  return score_sum;
}

double cda_rail::sim::ScoreSet::get_destination_score() const {
  double score_sum = 0;
  for (auto const& [train_name, score] : destination_scores) {
    score_sum = score_sum + score;
  }

  return score_sum;
}

const cda_rail::sim::ScoreSet&
cda_rail::sim::SolverResult::get_score_set() const {
  return scores;
}
