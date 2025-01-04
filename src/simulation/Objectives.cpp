#include "simulation/Objectives.hpp"

double
cda_rail::sim::collision_penalty(const TrainTrajectorySet&     traj_set,
                                 std::function<double(double)> dist_penalty_fct,
                                 double dist_safe_thres) {
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

      for (size_t timestep = std::max(first_step1, first_step2);
           timestep <= std::min(last_step1, last_step2);) {
        double dist =
            traj_set.train_distance((*train1).name, (*train2).name, timestep)
                .value();

        if (dist >= dist_safe_thres) {
          size_t guaranteed_safe_time = std::floor(
              (dist - dist_safe_thres) / (2 * instance.get_max_train_speed()));
          timestep = timestep + guaranteed_safe_time;
        } else {
          score = score + dist_penalty_fct(dist);
          timestep++;
        }
      }
    }
  }

  return score;
}

double cda_rail::sim::reciprocal_dist_penalty(double dist, double thres) {
  return (thres / dist - 1);
}
