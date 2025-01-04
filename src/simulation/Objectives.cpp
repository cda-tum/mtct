#include "simulation/Objectives.hpp"

double
cda_rail::sim::collision_penalty(const TrainTrajectorySet&     traj_set,
                                 std::function<double(double)> dist_penalty_fct,
                                 double safety_distance) {
  /**
   * Check all trains for violation in minimum distances
   * Train position is assumed to be the center of the train
   *
   * @param traj_set Set of train trajectories
   * @param dist_penalty_fct Penalty invoked when distance is smaller than
   * safety_distance Distance argument is normalized (0-1)
   * @param safety_distance  Distance between trains below which penalties are
   * applied
   */
  const SimulationInstance& instance   = traj_set.get_instance();
  const TrainList&          train_list = instance.timetable.get_train_list();
  double                    score      = 0;

  for (auto train1 = train_list.begin(); train1 != train_list.end(); train1++) {
    const TrainTrajectory& traj1       = traj_set.get_traj((*train1).name);
    size_t                 first_step1 = traj1.get_first_timestep();
    size_t                 last_step1  = traj1.get_last_timestep();

    for (auto train2 = train1 + 1; train2 != train_list.end(); train2++) {
      const TrainTrajectory& traj2       = traj_set.get_traj((*train2).name);
      size_t                 first_step2 = traj2.get_first_timestep();
      size_t                 last_step2  = traj2.get_last_timestep();

      if (last_step1 < first_step2 || last_step2 < first_step1)
        continue;

      double safe_dist =
          0.5 * (*train1).length + 0.5 * (*train2).length + safety_distance;

      if (2 * safe_dist < (*train1).max_speed + (*train2).max_speed)
        throw std::logic_error("Time resolution too low.");

      for (size_t timestep = std::max(first_step1, first_step2);
           timestep <= std::min(last_step1, last_step2);) {
        double dist =
            traj_set.train_distance((*train1).name, (*train2).name, timestep)
                .value();

        if (dist >= safe_dist) {
          // TODO: we could differentiate here between uni/bidirectional and the
          // two trains
          size_t guaranteed_safe_time =
              std::floor((dist - safe_dist) /
                         std::max((*train1).max_speed, (*train2).max_speed));
          timestep = timestep + std::max(guaranteed_safe_time, (size_t)1);
        } else {
          score = score + dist_penalty_fct(dist / safe_dist);
          timestep++;
        }
      }
    }
  }

  return score;
}

double cda_rail::sim::reciprocal_dist_penalty(double dist) {
  return (1 / dist - 1);
}
