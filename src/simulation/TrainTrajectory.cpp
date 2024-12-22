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

    EdgeTrajectory       edge_traj(instance, train, solution.v_targets,
                                   initial_edge_states.back());
    EdgeTransitionResult transition =
        edge_traj.enter_next_edge(switch_direction);

    // switch (transition.outcome) {
    // case OVERSPEED:
    //   // TODO: braking + jump back
    // case DEADEND:
    // // TODO: braking + waiting + jump back
    // case PLANNED_STOP:
    // // TODO: braking + waiting + jump back
    // case TIME_END:
    // // TODO: return
    // case NORMAL:
    // // TODO: continue
    // default:
    // }

    if (abort > 1000)
      throw exceptions::ConsistencyException(
          "Trajectory construction did not terminate.");
  }
}

// TODO
// cda_rail::SpeedTargets cda_rail::TrainTrajectory::match_velocity(
//     double target_speed, std::optional<ulong> hold_until_timestep) {}
//
// std::tuple<ulong, ulong> cda_rail::TrainTrajectory::find_braking_point(
//     double target_speed, std::optional<ulong> hold_until_timestep) {
//
// }

bool cda_rail::TrainTrajectory::is_feasible_braking_point(ulong  timestep,
                                                          double target_speed) {
  double abs_diff_to_target_speed =
      std::abs(get_state(timestep).speed - target_speed);
  if (timestep > edge_trajs.back().get_last_timestep() || timestep < 1)
    throw std::out_of_range("Timestep out of range.");

  double starting_speed  = get_state(timestep).speed;
  double speed_diff      = target_speed - starting_speed;
  double speed_diff_abs  = std::abs(speed_diff);
  bool   accel_direction = std::signbit(speed_diff);

  double accel;
  if (accel_direction) {
    accel = train.acceleration;
  } else {
    accel = train.deceleration;
  }

  ulong required_braking_time = std::ceil(speed_diff / accel);
  ulong speed_reached_at      = timestep + required_braking_time;

  // This is the definite integral under the braking curve
  // As defined in EdgeTrajectory speed changes are done at maximum acceleration
  double braking_distance =
      starting_speed * required_braking_time +
      std::copysign((required_braking_time - 1) * required_braking_time * accel,
                    speed_diff);
  // TODO
  // double distance_remaining =
  return false;
}

cda_rail::TrainState cda_rail::TrainTrajectory::get_state(ulong timestep) {
  if (timestep > edge_trajs.back().get_last_timestep() || timestep < 1)
    throw std::out_of_range("Timestep out of range.");

  EdgeTrajectory& relevant_trajectory =
      edge_trajs.at(get_relevant_trajectory(timestep));
  size_t trajectory_idx = timestep - relevant_trajectory.get_initial_timestep();

  return TrainState{
      .timestep    = timestep,
      .edge        = relevant_trajectory.get_edge(),
      .position    = relevant_trajectory.get_positions().at(trajectory_idx),
      .orientation = relevant_trajectory.get_orientation(),
      .speed       = relevant_trajectory.get_speeds().at(trajectory_idx),
  };
}

double cda_rail::TrainTrajectory::get_distance_to_end(ulong timestep) {
  size_t start_traj_idx = get_relevant_trajectory(timestep);

  double distance = 0;
  for (auto it = edge_trajs.begin() + start_traj_idx; it != edge_trajs.end();
       it++) {
  }
  // TODO
  return 0.0;
}

size_t cda_rail::TrainTrajectory::get_relevant_trajectory(ulong timestep) {
  if (timestep > edge_trajs.back().get_last_timestep() || timestep < 1)
    throw std::out_of_range("Timestep out of range.");

  for (auto it = edge_trajs.begin(); it != edge_trajs.end(); it++) {
    if ((*it).get_last_timestep() >= timestep) {
      return (size_t)std::distance(edge_trajs.begin(), it);
    }
  }
  return 0;
}

cda_rail::TrainState cda_rail::TrainTrajectory::read_initial_train_state() {
  cda_rail::Schedule train_schedule =
      instance.timetable.get_schedule(train.name);
  return TrainState{
      .timestep = (ulong)train_schedule.get_t_0(),
      .edge =
          instance.network.get_successors(train_schedule.get_entry()).front(),
      .position    = 0,
      .orientation = true,
      .speed       = train_schedule.get_v_0()};
}
