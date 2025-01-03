#include "simulation/TrainTrajectory.hpp"

cda_rail::sim::TrainTrajectory::TrainTrajectory(
    const SimulationInstance& instance, const Train& train,
    RoutingSolution init_solution)
    : instance(instance), train(train), solution(init_solution) {
  for (size_t abort = 0;; abort++) {
    TrainState initial_edge_state;
    if (edge_trajs.size() == 0) {
      initial_edge_state = read_initial_train_state();
    } else {
      double switch_direction =
          solution.switch_directions.at(edge_trajs.size());

      EdgeEntry entry = edge_trajs.back().enter_next_edge(switch_direction);

      switch (entry.outcome) {
      case OVERSPEED: {
        double prev_speed_limit =
            instance.network.get_edge(entry.new_state.value().edge).max_speed;
        BrakingPeriod braking_period = add_braking(prev_speed_limit, {}, {});
        backtrack_trajectory(std::get<0>(braking_period));
        continue;
      }
      case DEADEND: {
        std::optional<u_int64_t> reversal_time =
            solution.v_targets.find_next_reversal(
                edge_trajs.back().get_last_timestep());

        BrakingPeriod braking_period = add_braking(
            0, reversal_time.value_or(instance.n_timesteps - 1), {});
        backtrack_trajectory(std::get<0>(braking_period));
        continue;
      }
      case PLANNED_STOP: {
        ScheduledStop stop = edge_trajs.back().get_stop();
        if (std::find(visited_stops.begin(), visited_stops.end(), stop) ==
            visited_stops.end()) {
          // TODO: unsafe cast
          BrakingPeriod braking_period =
              add_braking(0, {}, stop.get_min_stopping_time());
          backtrack_trajectory(std::get<0>(braking_period));
          visited_stops.push_back(stop);
          continue;
        } else {
          initial_edge_state = entry.new_state.value();
        }
      }
      case TIME_END: {
        return;
      }
      case NORMAL: {
        initial_edge_state = entry.new_state.value();
      }
      }
    }

    SpeedTargets previous_targets(solution.v_targets);

    double curr_speed_limit =
        instance.network.get_edge(initial_edge_state.edge).max_speed;
    solution.v_targets.limit_speed_from(curr_speed_limit,
                                        initial_edge_state.timestep);

    edge_trajs.push_back(EdgeTrajectory(instance, train, solution.v_targets,
                                        initial_edge_state));

    // Restore original targets after leaving edge
    // TODO: Braking can shift traversal forwards when changing direction
    // so we sometimes constrain speeds unnecessarily
    if (u_int64_t last_step = edge_trajs.back().get_last_timestep();
        last_step < instance.n_timesteps - 1) {
      solution.v_targets.delete_range(last_step + 1, instance.n_timesteps - 1);
      solution.v_targets.insert(
          previous_targets.copy_range(last_step + 1, instance.n_timesteps - 1));
    }

    if (abort > 1000)
      throw exceptions::ConsistencyException(
          "Trajectory construction did not terminate.");
  }
}

void cda_rail::sim::TrainTrajectory::backtrack_trajectory(u_int64_t timestep) {
  size_t i = get_earliest_affected_trajectory(timestep);

  while (edge_trajs.size() > i) {
    edge_trajs.pop_back();
  }
}

cda_rail::sim::BrakingPeriod cda_rail::sim::TrainTrajectory::add_braking(
    double abs_target_speed, std::optional<u_int64_t> hold_until,
    std::optional<u_int64_t> hold_at_least) {
  if (!edge_trajs.back().get_traversal().has_value())
    throw exceptions::ConsistencyException("No traversal to brake for.");

  EdgeTraversal traversal = edge_trajs.back().get_traversal().value();
  double        target_speed =
      std::copysign(abs_target_speed, traversal.crossing_orientation);

  std::optional<BrakingPeriod> braking_period =
      find_latest_braking_period(target_speed);

  if (!braking_period.has_value())
    throw exceptions::ConsistencyException("No feasible braking period found.");

  u_int64_t start_braking, end_braking;
  std::tie(start_braking, end_braking) = braking_period.value();
  u_int64_t end_hold                   = end_braking;

  if (hold_until.has_value() && hold_until.value() > end_hold)
    end_hold = hold_until.value();
  if (hold_at_least.has_value() &&
      hold_at_least.value() + start_braking > end_hold)
    end_hold = hold_at_least.value() + start_braking;

  if (solution.v_targets.is_first_target(start_braking))
    start_braking = 0;

  solution.v_targets.set_range(start_braking, end_hold, target_speed);

  return std::tuple(start_braking, end_hold);
}

std::optional<cda_rail::sim::BrakingPeriod>
cda_rail::sim::TrainTrajectory::find_latest_braking_period(
    double target_speed) const {
  u_int64_t last_timestep  = edge_trajs.back().get_last_timestep();
  u_int64_t first_timestep = edge_trajs.front().get_first_timestep();

  for (u_int64_t start_braking = last_timestep; start_braking >= first_timestep;
       start_braking--) {
    std::optional<u_int64_t> end_braking =
        is_feasible_braking_point(start_braking, target_speed);

    if (end_braking.has_value())
      return std::tuple(start_braking, end_braking.value());

    if (start_braking == 0)
      break;
  }
  return {};
}

std::optional<u_int64_t>
cda_rail::sim::TrainTrajectory::is_feasible_braking_point(
    u_int64_t start_braking, double target_speed) const {
  double abs_diff_to_target_speed =
      std::abs(get_state(start_braking).speed - target_speed);
  if (start_braking > edge_trajs.back().get_last_timestep() ||
      start_braking < edge_trajs.front().get_first_timestep())
    throw std::out_of_range("Timestep out of range.");

  double starting_speed  = get_state(start_braking).speed;
  double speed_diff      = target_speed - starting_speed;
  double speed_diff_abs  = std::abs(speed_diff);
  bool   accel_direction = !std::signbit(speed_diff);

  double accel;
  if (accel_direction) {
    accel = train.acceleration;
  } else {
    accel = train.deceleration;
  }

  u_int64_t required_braking_time = std::ceil(speed_diff_abs / accel);

  // This is the definite integral under the braking curve
  // As defined in EdgeTrajectory speed changes are done at maximum acceleration
  // Zero-crossing and bidirectionality need to be considered
  double braking_dist =
      starting_speed * required_braking_time +
      (0.5 * std::copysign((required_braking_time - 1) *
                               (required_braking_time * accel),
                           speed_diff));

  u_int64_t end_braking = start_braking + required_braking_time;

  if (end_braking <= edge_trajs.back().get_last_timestep()) {
    return end_braking;
  } else {
    return {};
  }
}

cda_rail::sim::TrainState
cda_rail::sim::TrainTrajectory::get_state(u_int64_t timestep) const {
  if (timestep > edge_trajs.back().get_last_timestep())
    throw std::out_of_range("Timestep out of range.");

  const EdgeTrajectory& relevant_trajectory =
      edge_trajs.at(get_matching_trajectory(timestep));
  size_t trajectory_idx = timestep - relevant_trajectory.get_first_timestep();

  return TrainState{
      .timestep    = timestep,
      .edge        = relevant_trajectory.get_edge(),
      .position    = relevant_trajectory.get_positions().at(trajectory_idx),
      .orientation = relevant_trajectory.get_orientation(),
      .speed       = relevant_trajectory.get_speeds().at(trajectory_idx),
  };
}

size_t cda_rail::sim::TrainTrajectory::get_matching_trajectory(
    u_int64_t timestep) const {
  for (auto it = edge_trajs.begin(); it != edge_trajs.end(); it++) {
    if ((*it).get_first_timestep() <= timestep &&
        (*it).get_last_timestep() >= timestep) {
      return (size_t)std::distance(edge_trajs.begin(), it);
    }
  }
  throw std::out_of_range("Timestep not contained in any trajectory.");
}

size_t cda_rail::sim::TrainTrajectory::get_earliest_affected_trajectory(
    u_int64_t timestep) const {
  for (auto it = edge_trajs.begin(); it != edge_trajs.end(); it++) {
    if ((*it).get_last_timestep() >= timestep) {
      return (size_t)std::distance(edge_trajs.begin(), it);
    }
  }
  throw std::out_of_range("No affected trajectory found.");
}

size_t cda_rail::sim::TrainTrajectory::get_first_timestep() const {
  return edge_trajs.front().get_first_timestep();
}

size_t cda_rail::sim::TrainTrajectory::get_last_timestep() const {
  return edge_trajs.back().get_last_timestep();
}

cda_rail::sim::TrainState
cda_rail::sim::TrainTrajectory::read_initial_train_state() const {
  cda_rail::Schedule train_schedule =
      instance.timetable.get_schedule(train.name);
  return TrainState{
      .timestep = (u_int64_t)train_schedule.get_t_0(),
      .edge =
          instance.network.get_successors(train_schedule.get_entry()).front(),
      .position    = 0,
      .orientation = true,
      .speed       = train_schedule.get_v_0()};
}

void cda_rail::sim::TrainTrajectory::check_speed_limits() const {
  for (auto traj : edge_trajs) {
    traj.check_speed_limits();
  }
}
