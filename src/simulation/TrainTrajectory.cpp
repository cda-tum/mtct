#include "simulation/TrainTrajectory.hpp"

cda_rail::TrainTrajectory::TrainTrajectory(SimulationInstance& instance,
                                           Train&              train,
                                           RoutingSolution     solution)
    : instance(instance), train(train), solution(solution) {
  initial_edge_states.push_back(read_initial_train_state());

  for (size_t abort = 0;; abort++) {
    double switch_direction =
        solution.switch_directions.at(initial_edge_states.size() - 1);

    SpeedTargets working_targets(solution.v_targets);
    double       speed_limit =
        instance.network.get_edge(initial_edge_states.back().edge).max_speed;
    working_targets.limit_speed_from(speed_limit,
                                     initial_edge_states.back().timestep);

    edge_trajs.push_back(EdgeTrajectory(instance, train, solution.v_targets,
                                        initial_edge_states.back()));
    EdgeEntry transition = edge_trajs.back().enter_next_edge(switch_direction);

    // switch (transition.outcome) {
    // case OVERSPEED:
    //   // TODO: braking + merge targ + jump back
    // case DEADEND:
    // // TODO: braking + wait till reverse target + merge targ + jump back
    // case PLANNED_STOP:
    // // TODO: braking + wait fixed time + merge targ + jump back
    // case TIME_END:
    // // TODO: return
    // case NORMAL:
    // // TODO: continue
    // default:
    // }

    // TODO: Merge speed targets

    // TODO: retry calculating edge

    if (abort > 1000)
      throw exceptions::ConsistencyException(
          "Trajectory construction did not terminate.");
  }
}

cda_rail::BrakingPeriod cda_rail::TrainTrajectory::brake_before_transit(
    double abs_target_speed, std::optional<ulong> hold_until,
    std::optional<ulong> hold_at_least) {
  if (!edge_trajs.back().get_transition().has_value())
    throw exceptions::ConsistencyException("No transition to brake for.");

  EdgeExit transition = edge_trajs.back().get_transition().value();
  double   target_speed =
      std::copysign(abs_target_speed, transition.traversal_direction);

  std::optional<BrakingPeriod> braking_period =
      find_latest_braking_period(target_speed);

  if (!braking_period.has_value())
    throw exceptions::ConsistencyException("No feasible braking period found.");

  ulong start_braking, end_braking;
  std::tie(start_braking, end_braking) = braking_period.value();
  ulong end_hold                       = end_braking;

  if (hold_until.has_value() && hold_until.value() > end_hold)
    end_hold = hold_until.value();
  if (hold_at_least.has_value() &&
      hold_at_least.value() + start_braking > end_hold)
    end_hold = hold_at_least.value() + start_braking;

  solution.v_targets.delete_range(start_braking, end_hold);

  solution.v_targets.targets.insert_or_assign(start_braking, target_speed);

  return braking_period.value();
}

std::optional<cda_rail::BrakingPeriod>
cda_rail::TrainTrajectory::find_latest_braking_period(
    double target_speed) const {
  for (ulong start_braking = edge_trajs.back().get_last_timestep();
       start_braking >= 0; start_braking--) {
    std::optional<ulong> end_braking =
        is_feasible_braking_point(start_braking, target_speed);
    if (end_braking.has_value())
      return std::tuple(start_braking, end_braking.value());
  }
  return {};
}

std::optional<ulong> cda_rail::TrainTrajectory::is_feasible_braking_point(
    ulong start_braking, double target_speed) const {
  double abs_diff_to_target_speed =
      std::abs(get_state(start_braking).speed - target_speed);
  if (start_braking > edge_trajs.back().get_last_timestep() ||
      start_braking < 0)
    throw std::out_of_range("Timestep out of range.");

  double starting_speed  = get_state(start_braking).speed;
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

  // This is the definite integral under the braking curve
  // As defined in EdgeTrajectory speed changes are done at maximum acceleration
  // Zero-crossing and bidirectionality need to be considered
  double braking_dist =
      starting_speed * required_braking_time +
      std::copysign((required_braking_time - 1) * required_braking_time * accel,
                    speed_diff);

  // Distance is defined from the train point of view on a fixed path
  double dist_to_transition = distance_to_last_transition(start_braking);
  // TODO: Deviates from matlab version
  double dist_after_braking = dist_to_transition - braking_dist;

  ulong end_braking = start_braking + required_braking_time;

  if (end_braking < edge_trajs.back().get_last_timestep() + 1 &&
      dist_after_braking > 0) {
    return end_braking;
  } else {
    return {};
  }
}

double
cda_rail::TrainTrajectory::distance_to_last_transition(ulong timestep) const {
  size_t start_traj_idx = get_relevant_trajectory(timestep);

  if (!edge_trajs.back().get_transition().has_value())
    throw exceptions::ConsistencyException(
        "Last edge trajectory has no transition.");

  double distance = 0;
  for (auto it = edge_trajs.begin() + start_traj_idx; it != edge_trajs.end();
       it++) {
    double                     start_position, end_position;
    const std::vector<double>& positions = (*it).get_positions();

    if (it == edge_trajs.begin() + start_traj_idx) {
      start_position = positions.front();
    } else {
      start_position = (double)(*it--).get_transition().value().exit_point;
    }

    end_position = (double)(*it).get_transition().value().exit_point;

    distance += (end_position - start_position) *
                (2 * ((double)(*it).get_orientation()) - 1);
  }

  return distance;
}

cda_rail::TrainState
cda_rail::TrainTrajectory::get_state(ulong timestep) const {
  if (timestep > edge_trajs.back().get_last_timestep() || timestep < 1)
    throw std::out_of_range("Timestep out of range.");

  const EdgeTrajectory& relevant_trajectory =
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

size_t
cda_rail::TrainTrajectory::get_relevant_trajectory(ulong timestep) const {
  if (timestep > edge_trajs.back().get_last_timestep() || timestep < 1)
    throw std::out_of_range("Timestep out of range.");

  for (auto it = edge_trajs.begin(); it != edge_trajs.end(); it++) {
    if ((*it).get_last_timestep() >= timestep) {
      return (size_t)std::distance(edge_trajs.begin(), it);
    }
  }
  return 0;
}

cda_rail::TrainState
cda_rail::TrainTrajectory::read_initial_train_state() const {
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
