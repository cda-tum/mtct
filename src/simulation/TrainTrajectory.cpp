#include "simulation/TrainTrajectory.hpp"

cda_rail::TrainTrajectory::TrainTrajectory(const SimulationInstance& instance,
                                           const Train&              train,
                                           RoutingSolution           solution)
    : solution(solution) {
  initial_edge_states.push_back(read_initial_train_state(instance, train));

  for (size_t abort = 0;; abort++) {
    double switch_direction =
        solution.switch_directions.at(initial_edge_states.size() - 1);
    EdgeTrajectory edge_traj(instance, train, solution.v_targets,
                             initial_edge_states.back());
    EdgeTransition transition =
        edge_traj.get_transition(instance, train, switch_direction);

    switch (transition.outcome) {
    case OVERSPEED:
    // TODO: braking + jump back
    case DEADEND:
    // TODO: braking + waiting + jump back
    case PLANNED_STOP:
    // TODO: braking + waiting + jump back
    case TIME_END:
    // TODO: return
    case NORMAL:
    // TODO: continue
    default:
    }

    if (abort > 1000)
      throw exceptions::ConsistencyException(
          "Trajectory construction did not terminate.");
  }
}

cda_rail::InitialEdgeState
cda_rail::read_initial_train_state(const SimulationInstance& instance,
                                   const Train&              train) {
  cda_rail::Schedule train_schedule =
      instance.timetable.get_schedule(train.name);
  return InitialEdgeState{
      .timestep = (ulong)train_schedule.get_t_0(),
      .edge =
          instance.network.get_successors(train_schedule.get_entry()).front(),
      .position    = 0,
      .orientation = true,
      .speed       = train_schedule.get_v_0()};
}
