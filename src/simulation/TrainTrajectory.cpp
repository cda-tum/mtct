#include "simulation/TrainTrajectory.hpp"

cda_rail::TrainTrajectory::TrainTrajectory(SimulationInstance& instance,
                                           Train&              train,
                                           RoutingSolution     solution)
    : instance(instance), train(train), solution(solution) {
  initial_edge_states.push_back(read_initial_train_state());

  for (size_t abort = 0;; abort++) {
    double switch_direction =
        solution.switch_directions.at(initial_edge_states.size() - 1);

    // TODO: clamp speed
    // TODO: adjust starting target

    EdgeTrajectory edge_traj(instance, train, solution.v_targets,
                             initial_edge_states.back());
    EdgeTransition transition = edge_traj.get_transition(switch_direction);

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

cda_rail::SpeedTargets cda_rail::TrainTrajectory::match_velocity(
    EdgeTransition transition, SpeedTargets v_targets, double target_speed,
    std::optional<ulong> hold_until_timestep) {}

std::tuple<ulong, ulong> cda_rail::TrainTrajectory::find_braking_point(
    EdgeTransition transition, SpeedTargets v_targets, double target_speed,
    std::optional<ulong> hold_until_timestep) {}

cda_rail::InitialEdgeState
cda_rail::TrainTrajectory::read_initial_train_state() {
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
