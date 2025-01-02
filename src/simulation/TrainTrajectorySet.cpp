#include "simulation/TrainTrajectorySet.hpp"

cda_rail::TrainTrajectorySet::TrainTrajectorySet(
    const SimulationInstance& instance, const RoutingSolutionSet& solution_set)
    : instance(instance) {
  const TrainList& trainlist = instance.timetable.get_train_list();
  for (auto solution : solution_set.solutions) {
    TrainTrajectory traj{instance, trainlist.get_train(solution.first),
                         solution.second};
    trajectories.insert({solution.first, traj});
  }
}

size_t cda_rail::TrainTrajectorySet::size() const {
  return trajectories.size();
}

void cda_rail::TrainTrajectorySet::export_csv(
    const std::filesystem::path& p) const {
  std::ofstream    csvfile(p);
  const TrainList& train_list = instance.timetable.get_train_list();

  for (auto traj : trajectories) {
    for (size_t timestep = traj.second.get_first_timestep();
         timestep <= traj.second.get_last_timestep(); timestep++) {
      csvfile << train_list.get_train_index(traj.first) << ", ";
      csvfile << traj.first << ", ";
      TrainState state = traj.second.get_state(timestep);
      csvfile << timestep << ", ";
      csvfile << state.edge << ", ";
      csvfile << state.position << "\n";
    }
  }

  csvfile.close();
}
