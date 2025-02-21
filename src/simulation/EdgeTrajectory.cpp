#include "simulation/EdgeTrajectory.hpp"

#include <algorithm>

cda_rail::sim::EdgeEntry::EdgeEntry(
    cda_rail::sim::EdgeEntryOutcome          outcome,
    std::optional<cda_rail::sim::TrainState> new_state)
    : outcome(outcome), new_state(new_state) {
  bool requires_state = (outcome == NORMAL || outcome == OVERSPEED);
  if (requires_state && !new_state.has_value() ||
      !requires_state && new_state.has_value())
    throw std::logic_error("Improper result of edge entry.");
}

cda_rail::sim::EdgeTrajectory::EdgeTrajectory(
    const SimulationInstance& instance, const Train& train,
    SpeedTargets& v_targets, TrainState initial_state)
    : instance_r(std::ref(instance)), train_r(std::ref(train)),
      first_timestep(initial_state.timestep), edge(initial_state.edge),
      orientation(initial_state.orientation) {
  double edge_length         = instance_r.get().network.get_edge(edge).length;
  double edge_length_divisor = 1 / edge_length;
  // TODO: narrowing cast should be avoided
  size_t exit_time =
      instance_r.get().timetable.get_schedule(train_r.get().name).get_t_n();

  speeds.push_back(initial_state.speed);
  positions.push_back(initial_state.position);

  for (int timestep = first_timestep + 1; timestep <= exit_time; timestep++) {
    double speed           = speeds.back();
    double speed_disparity = v_targets.find_target_speed(timestep - 1) - speed;
    double acceleration;
    if (speed_disparity >= 0) {
      acceleration = std::min(speed_disparity, train_r.get().acceleration);
    } else {
      acceleration = std::max(speed_disparity, -train_r.get().deceleration);
    }

    speeds.push_back(speeds.back() + acceleration);
    positions.push_back(positions.back() + ((2 * ((double)orientation) - 1) *
                                            speed * edge_length_divisor));

    if (positions.back() > 1 || positions.back() < 0) {
      last_timestep = first_timestep + positions.size() - 2;
      traversal     = determine_exit(instance_r.get().network,
                                     TrainState{
                                         .timestep    = last_timestep + 1,
                                         .edge        = edge,
                                         .position    = positions.back(),
                                         .orientation = orientation,
                                         .speed       = speeds.back(),
                                 });
      positions.pop_back();
      speeds.pop_back();
      return;
    }
  }

  last_timestep = first_timestep + positions.size() - 1;
  traversal     = {};
}

cda_rail::sim::EdgeEntry
cda_rail::sim::EdgeTrajectory::enter_next_edge(double switch_direction) const {
  if (!traversal.has_value()) {
    return EdgeEntry{TIME_END, {}};
  }

  // traversal along edges until no longer overshooting on first step
  //
  //          +----> EdgeTrajectory ---+ EdgeTraj()
  //          |                        |
  //          |                        v
  //     TrainState                TrainState
  //     initial_state ---------> overshot_state
  //          ^                        |
  //          |                        |
  //          +------- EdgeTraversal <-+
  //  determine_first_state()     determine_exit()
  //

  EdgeTraversal edge_exit = traversal.value();
  TrainState    new_state;
  for (bool overshot = true; overshot != false;) {
    // TODO: select separate direction for each iteration
    // TODO: trains should be able to exit the network at their designated node
    std::optional<TrainState> new_state_result = determine_first_state(
        instance_r.get().network, edge_exit, switch_direction);

    if (!new_state_result.has_value()) {
      return EdgeEntry{DEADEND, {}};
    } else {
      new_state = new_state_result.value();
    }

    overshot = (new_state.position < 0 || new_state.position > 1);

    if (overshot) {
      edge_exit = determine_exit(instance_r.get().network, new_state);
    }
  }

  EdgeEntryOutcome outcome;
  if (std::abs(edge_exit.speed) >
      instance_r.get().network.get_edge(new_state.edge).max_speed) {
    outcome = OVERSPEED;
  } else {
    outcome = NORMAL;
  }

  return EdgeEntry{outcome, new_state};
}

std::optional<cda_rail::sim::TrainState>
cda_rail::sim::determine_first_state(const cda_rail::Network&     network,
                                     cda_rail::sim::EdgeTraversal traversal,
                                     double switch_direction) {
  std::vector<size_t> viable_next_edges;
  if (traversal.from_exit_point) {
    viable_next_edges = network.get_successors(traversal.from_edge);
  } else {
    viable_next_edges = network.get_predecessors(traversal.from_edge);
  }

  if (viable_next_edges.size() < 1)
    return {};

  size_t next_edge = viable_next_edges.at(
      std::round(switch_direction * (viable_next_edges.size() - 1)));

  bool edge_entry_point =
      (network.get_edge(next_edge).target == traversal.vertex);

  double edge_entry_position;
  if (edge_entry_point) {
    edge_entry_position = 1 - std::abs(traversal.leftover_movement /
                                       network.get_edge(next_edge).length);
  } else {
    edge_entry_position = std::abs(traversal.leftover_movement /
                                   network.get_edge(next_edge).length);
  }

  return cda_rail::sim::TrainState{
      .timestep = traversal.from_timestep + 1,
      .edge     = next_edge,
      .position = edge_entry_position,
      .orientation =
          !(edge_entry_point != !traversal.crossing_orientation), // XNOR
      .speed = traversal.speed,
  };
}

cda_rail::sim::EdgeTraversal
cda_rail::sim::determine_exit(const cda_rail::Network&  network,
                              cda_rail::sim::TrainState overshot_state) {
  double edge_length = network.get_edge(overshot_state.edge).length;
  bool   exit_point  = (overshot_state.position > 1);
  size_t vertex;
  bool   crossing_orientation;
  double leftover_movement;

  if (exit_point) {
    return cda_rail::sim::EdgeTraversal{
        // Forward exit in edge direction
        .from_timestep        = overshot_state.timestep - 1,
        .from_edge            = overshot_state.edge,
        .from_exit_point      = exit_point,
        .vertex               = network.get_edge(overshot_state.edge).target,
        .crossing_orientation = overshot_state.orientation,
        .leftover_movement    = (overshot_state.position - 1) * edge_length,
        .speed                = overshot_state.speed,
    };
  } else {
    return cda_rail::sim::EdgeTraversal{
        // Backward exit in edge direction
        .from_timestep        = overshot_state.timestep - 1,
        .from_edge            = overshot_state.edge,
        .from_exit_point      = exit_point,
        .vertex               = network.get_edge(overshot_state.edge).source,
        .crossing_orientation = !overshot_state.orientation,
        .leftover_movement    = -overshot_state.position * edge_length,
        .speed                = overshot_state.speed,
    };
  }
}

void cda_rail::sim::EdgeTrajectory::check_speed_limits() const {
  double speed_limit =
      std::max(instance_r.get().network.get_edge(edge).max_speed,
               train_r.get().max_speed);

  if (positions.size() > 1) {
    double edge_length = instance_r.get().network.get_edge(edge).length;
    for (auto pos = positions.begin() + 1; pos != positions.end(); pos++) {
      if (std::abs((*pos) - *(pos - 1)) * edge_length > speed_limit)
        throw std::logic_error("Overspeed detected.");
    }
  }

  for (auto speed : speeds) {
    if (std::abs(speed) > speed_limit)
      throw std::logic_error("Overspeed detected.");
  }
}

cda_rail::ScheduledStop cda_rail::sim::EdgeTrajectory::get_stop() const {
  for (auto stop : instance_r.get()
                       .timetable.get_schedule(train_r.get().name)
                       .get_stops()) {
    auto stop_station =
        instance_r.get().timetable.get_station_list().get_station(
            stop.get_station_name());

    if (std::find(stop_station.tracks.begin(), stop_station.tracks.end(),
                  edge) != stop_station.tracks.end())
      return stop;
  }
  throw std::invalid_argument("No associated scheduled stop found.");
}

u_int64_t cda_rail::sim::EdgeTrajectory::get_first_timestep() const {
  return first_timestep;
}

u_int64_t cda_rail::sim::EdgeTrajectory::get_last_timestep() const {
  return last_timestep;
}

size_t cda_rail::sim::EdgeTrajectory::get_edge() const { return edge; }

bool cda_rail::sim::EdgeTrajectory::get_orientation() const {
  return orientation;
}

const std::vector<double>&
cda_rail::sim::EdgeTrajectory::get_positions() const {
  return positions;
}

const std::vector<double>& cda_rail::sim::EdgeTrajectory::get_speeds() const {
  return speeds;
}

std::optional<cda_rail::sim::EdgeTraversal>
cda_rail::sim::EdgeTrajectory::get_traversal() const {
  return traversal;
}
