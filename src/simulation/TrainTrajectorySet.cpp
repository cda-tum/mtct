#include "simulation/TrainTrajectorySet.hpp"

cda_rail::TrainTrajectorySet::TrainTrajectorySet(
    const SimulationInstance& instance,
    const RoutingSolutionSet& solution_set) {
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
