#include "simulation/EdgeTrajectory.hpp"

#include <algorithm>

cda_rail::EdgeEntry::EdgeEntry(cda_rail::EdgeEntryOutcome          outcome,
                               std::optional<cda_rail::TrainState> new_state)
    : outcome(outcome), new_state(new_state) {
  bool requires_state =
      (outcome == NORMAL || outcome == OVERSPEED || outcome == PLANNED_STOP);
  if (requires_state && !new_state.has_value() ||
      !requires_state && new_state.has_value())
    throw std::invalid_argument("Improper result of edge entry.");
}

cda_rail::EdgeTrajectory::EdgeTrajectory(SimulationInstance& instance,
                                         Train& train, SpeedTargets& v_targets,
                                         TrainState initial_state)
    : instance(instance), train(train),
      initial_timestep(initial_state.timestep), edge(initial_state.edge),
      orientation(initial_state.orientation) {
  double edge_length         = instance.network.get_edge(edge).length;
  double edge_length_divisor = 1 / edge_length;

  speeds.push_back(initial_state.speed);
  positions.push_back(initial_state.position);

  for (int timestep = initial_timestep + 1; timestep < instance.n_timesteps;
       timestep++) {
    double speed_disparity =
        v_targets.find_target_speed(timestep - 1) - speeds.back();
    double acceleration;
    if (speed_disparity >= 0) {
      acceleration = std::min(speed_disparity, train.acceleration);
    } else {
      acceleration = std::max(speed_disparity, -train.deceleration);
    }
    speeds.push_back(speeds.back() + acceleration);
    positions.push_back(positions.back() +
                        ((2 * ((double)orientation) - 1) * speeds.back() *
                         edge_length_divisor));

    if (positions.back() > 1 || positions.back() < 0) {
      last_timestep = initial_timestep + positions.size() - 2;
      traversal =
          determine_exit(instance.network, TrainState{
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

  last_timestep = initial_timestep + positions.size() - 1;
  traversal     = {};
}

cda_rail::EdgeEntry
cda_rail::EdgeTrajectory::enter_next_edge(double switch_direction) const {
  if (!traversal.has_value())
    return EdgeEntry{TIME_END, {}};

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
    std::optional<TrainState> new_state_result =
        determine_first_state(instance.network, edge_exit, switch_direction);

    if (!new_state_result.has_value()) {
      return EdgeEntry{DEADEND, {}};
    } else {
      new_state = new_state_result.value();
    }

    overshot = (new_state.position < 0 || new_state.position > 1);

    if (overshot) {
      edge_exit = determine_exit(instance.network, new_state);
    }
  }

  EdgeEntryOutcome outcome;
  if (std::abs(edge_exit.speed) >
      instance.network.get_edge(new_state.edge).max_speed) {
    outcome = OVERSPEED;
  } else if (is_planned_stop()) {
    outcome = PLANNED_STOP;
  } else {
    outcome = NORMAL;
  }

  return EdgeEntry{outcome, new_state};
}

std::optional<cda_rail::TrainState>
cda_rail::determine_first_state(const cda_rail::Network& network,
                                cda_rail::EdgeTraversal  traversal,
                                double                   switch_direction) {
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

  return cda_rail::TrainState{
      .timestep = traversal.from_timestep + 1,
      .edge     = next_edge,
      .position = edge_entry_position,
      .orientation =
          !(edge_entry_point != !traversal.crossing_orientation), // XNOR
      .speed = traversal.speed,
  };
}

cda_rail::EdgeTraversal
cda_rail::determine_exit(const cda_rail::Network& network,
                         cda_rail::TrainState     overshot_state) {
  double edge_length = network.get_edge(overshot_state.edge).length;
  bool   exit_point  = (overshot_state.position > 1);
  size_t vertex;
  bool   crossing_orientation;
  double leftover_movement;

  if (exit_point) {
    return cda_rail::EdgeTraversal{
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
    return cda_rail::EdgeTraversal{
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

bool cda_rail::EdgeTrajectory::is_planned_stop() const {
  for (auto stop : instance.timetable.get_schedule(train.name).get_stops()) {
    auto stop_station = instance.timetable.get_station_list().get_station(
        stop.get_station_name());

    if (std::find(stop_station.tracks.begin(), stop_station.tracks.end(),
                  edge) != stop_station.tracks.end())
      return true;
  }

  return false;
}

cda_rail::ScheduledStop cda_rail::EdgeTrajectory::get_stop() const {
  for (auto stop : instance.timetable.get_schedule(train.name).get_stops()) {
    auto stop_station = instance.timetable.get_station_list().get_station(
        stop.get_station_name());

    if (std::find(stop_station.tracks.begin(), stop_station.tracks.end(),
                  edge) != stop_station.tracks.end())
      return stop;
  }
  throw std::invalid_argument("No associated scheduled stop found.");
}

u_int64_t cda_rail::EdgeTrajectory::get_initial_timestep() const {
  return initial_timestep;
}

u_int64_t cda_rail::EdgeTrajectory::get_last_timestep() const {
  return last_timestep;
}

u_int64_t cda_rail::EdgeTrajectory::get_edge() const { return edge; }

bool cda_rail::EdgeTrajectory::get_orientation() const { return orientation; }

const std::vector<double>& cda_rail::EdgeTrajectory::get_positions() const {
  return positions;
}

const std::vector<double>& cda_rail::EdgeTrajectory::get_speeds() const {
  return speeds;
}

const std::optional<cda_rail::EdgeTraversal>&
cda_rail::EdgeTrajectory::get_traversal() const {
  return traversal;
}
