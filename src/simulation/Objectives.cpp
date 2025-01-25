#include "simulation/Objectives.hpp"

double cda_rail::sim::combined_objective(const TrainTrajectorySet& traj_set) {
  /**
   * Combined objectives
   */
  return collision_penalty(traj_set) + destination_penalty(traj_set) +
         stop_penalty(traj_set);
}

double cda_rail::sim::collision_penalty(const TrainTrajectorySet& traj_set) {
  /**
   * Check all trains for violation in minimum distances
   * Train position is assumed to be the center of the train
   *
   * @param traj_set Set of train trajectories
   * @param dist_penalty_fct Penalty invoked when distance is smaller than
   * safety_distance Distance argument is normalized (0-1)
   * @param safety_distance  Distance between trains below which penalties are
   * applied
   * @return Normalized penalty score from 0 to 1, lower is better
   */

  constexpr double safety_distance = 100;

  const SimulationInstance& instance   = traj_set.get_instance();
  const TrainList&          train_list = instance.timetable.get_train_list();
  double                    score      = 0;

  for (auto train1 = train_list.begin(); train1 != train_list.end(); train1++) {
    size_t first_step1, last_step1;
    if (const auto traj1 = traj_set.get_traj((*train1).name)) {
      first_step1 = traj1.value().get_first_timestep();
      last_step1  = traj1.value().get_last_timestep();
    } else {
      continue;
    }

    for (auto train2 = train1 + 1; train2 != train_list.end(); train2++) {
      size_t first_step2, last_step2;
      if (const auto traj2 = traj_set.get_traj((*train2).name)) {
        first_step2 = traj2.value().get_first_timestep();
        last_step2  = traj2.value().get_last_timestep();
      } else {
        continue;
      }

      if (last_step1 < first_step2 || last_step2 < first_step1)
        continue;

      double required_dist =
          0.5 * (*train1).length + 0.5 * (*train2).length + safety_distance;

      if (2 * required_dist < (*train1).max_speed + (*train2).max_speed)
        throw std::logic_error("Time resolution too low.");

      for (size_t timestep = std::max(first_step1, first_step2);
           timestep <= std::min(last_step1, last_step2);) {
        double dist =
            traj_set.train_distance((*train1).name, (*train2).name, timestep)
                .value();

        if (dist >= required_dist) {
          double max_approach_speed = (*train1).max_speed + (*train2).max_speed;
          double min_time_to_collision =
              std::max((dist - required_dist), 0.0) / max_approach_speed;
          size_t guaranteed_safe_time = std::floor(min_time_to_collision);
          timestep = timestep + std::max(guaranteed_safe_time, (size_t)1);
        } else {
          // TODO: normalization does not work since the amount of timesteps
          // checked is changing
          double penalty = 1 - (dist / required_dist);
          score          = score + penalty;
          timestep++;
        }
      }
    }
  }

  size_t n_pairs = (train_list.size() * (train_list.size() - 1)) / 2;
  return score / n_pairs;
}

double cda_rail::sim::destination_penalty(const TrainTrajectorySet& traj_set) {
  /**
   * Penalize all trains for the distance from their scheduled exit at their
   * final position Train position is assumed to be the center of the train
   *
   * @param traj_set Set of train trajectories
   * @return Normalized penalty score from 0 to 1, lower is better
   */

  const SimulationInstance& instance   = traj_set.get_instance();
  const TrainList&          train_list = instance.timetable.get_train_list();
  double                    score      = 0;

  for (auto train = train_list.begin(); train != train_list.end(); train++) {
    if (const auto traj_opt = traj_set.get_traj((*train).name)) {
      score = score + destination_penalty(traj_opt.value());
    } else {
      continue;
    }
  }

  return score / traj_set.size();
}

double cda_rail::sim::destination_penalty(const TrainTrajectory& traj) {
  /**
   * Penalize a train for the distance from their scheduled exit at their
   * final position Train position is assumed to be the center of the train
   *
   * @param traj train trajectory
   * @return Normalized penalty score from 0 to 1, lower is better
   */

  const SimulationInstance& instance   = traj.get_instance();
  const TrainList&          train_list = instance.timetable.get_train_list();
  double                    score      = 0;

  size_t dest_vertex =
      instance.timetable.get_schedule(traj.get_train().name).get_exit();
  double max_dist =
      *std::max_element(instance.shortest_paths.at(dest_vertex).begin(),
                        instance.shortest_paths.at(dest_vertex).end());
  double dist =
      traj.train_vertex_distance(dest_vertex, traj.get_last_timestep()).value();

  // TODO: Penalize wrong exit speed as well as position
  return dist / max_dist;
}

double cda_rail::sim::stop_penalty(const TrainTrajectorySet& traj_set) {
  /**
   * Penalize trains not visiting their scheduled stop
   *
   * @param traj_set Set of train trajectories
   * @return Normalized penalty score from 0 to 1, lower is better
   */

  const SimulationInstance& instance   = traj_set.get_instance();
  const TrainList&          train_list = instance.timetable.get_train_list();
  double                    score      = 0;

  if (traj_set.size() < 1)
    throw std::invalid_argument("Cannot evaluate empty trajectory set.");

  for (auto train = train_list.begin(); train != train_list.end(); train++) {
    if (const auto traj_opt = traj_set.get_traj((*train).name)) {
      score = score + stop_penalty(traj_opt.value());
    } else {
      continue;
    }
  }

  return score / traj_set.size();
}

double cda_rail::sim::stop_penalty(const TrainTrajectory& traj) {
  /**
   * Penalize train not visiting their scheduled stop
   *
   * @param traj train trajectory
   * @return Normalized penalty score from 0 to 1, lower is better
   */
  size_t n_visited_stops   = traj.get_visited_stop_amount();
  size_t n_scheduled_stops = traj.get_instance()
                                 .timetable.get_schedule(traj.get_train().name)
                                 .get_stops()
                                 .size();

  if (n_visited_stops > n_scheduled_stops)
    throw std::logic_error("Visited more stops than scheduled.");

  if (n_scheduled_stops == 0)
    return 0;

  return (n_scheduled_stops - n_visited_stops) / n_scheduled_stops;
}
