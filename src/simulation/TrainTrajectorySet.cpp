#include "simulation/TrainTrajectorySet.hpp"

cda_rail::sim::TrainTrajectorySet::TrainTrajectorySet(
    const SimulationInstance& instance, const RoutingSolutionSet& solution_set)
    : instance(instance) {
  const TrainList& trainlist = instance.timetable.get_train_list();
  for (auto solution : solution_set.solutions) {
    TrainTrajectory traj{instance, trainlist.get_train(solution.first),
                         solution.second};
    trajectories.insert({solution.first, traj});
  }
}

std::optional<double> cda_rail::sim::TrainTrajectorySet::train_distance(
    std::string train1, std::string train2, size_t timestep) const {
  if (instance.bidirectional_travel)
    throw std::runtime_error(
        "Distance metric only works with unidirectional traffic.");

  TrainState state1, state2;
  {
    std::optional<TrainState> state_opt1 =
        trajectories.at(train1).get_state(timestep);
    std::optional<TrainState> state_opt2 =
        trajectories.at(train2).get_state(timestep);

    if (!state_opt1.has_value() || !state_opt2.has_value()) {
      return {};
    } else {
      state1 = state_opt1.value();
      state2 = state_opt2.value();
    }
  }

  double edge_length1 = instance.network.get_edge(state1.edge).length;

  if (state1.edge == state2.edge)
    return std::abs(state1.position - state2.position) * edge_length1;

  double edge_length2 = instance.network.get_edge(state2.edge).length;

  double aepsp_metric1 =
      instance.shortest_paths.at(state1.edge).at(state2.edge);
  double aepsp_metric2 =
      instance.shortest_paths.at(state2.edge).at(state1.edge);

  double dist1 = aepsp_metric1 - state1.position * edge_length1 -
                 (1 - state2.position) * edge_length2;
  double dist2 = aepsp_metric2 - state2.position * edge_length2 -
                 (1 - state1.position) * edge_length1;

  return std::min(dist1, dist2);
}

void cda_rail::sim::TrainTrajectorySet::export_csv(
    const std::filesystem::path& p) const {
  std::ofstream csvfile(p);
  csvfile << "train_idx,train_name,timestep,edge_idx,edge_src_node,edge_dst_"
             "node,edge_pos\n";

  const TrainList& train_list = instance.timetable.get_train_list();

  for (auto traj : trajectories) {
    for (size_t timestep = traj.second.get_first_timestep();
         timestep <= traj.second.get_last_timestep(); timestep++) {
      csvfile << train_list.get_train_index(traj.first) << ",";
      csvfile << traj.first << ",";

      csvfile << timestep << ",";

      TrainState  state = traj.second.get_state(timestep).value();
      const Edge& edge  = instance.network.get_edge(state.edge);

      csvfile << state.edge << ",";
      csvfile << instance.network.get_vertex(edge.source).name << ",";
      csvfile << instance.network.get_vertex(edge.target).name << ",";

      csvfile << state.position << "\n";
    }
  }

  csvfile.close();
}

void cda_rail::sim::TrainTrajectorySet::check_speed_limits() const {
  for (auto traj : trajectories) {
    traj.second.check_speed_limits();
  }
}

const cda_rail::sim::TrainTrajectory&
cda_rail::sim::TrainTrajectorySet::get_traj(std::string train_name) const {
  return trajectories.at(train_name);
}

const cda_rail::sim::SimulationInstance&
cda_rail::sim::TrainTrajectorySet::get_instance() const {
  return instance;
}

size_t cda_rail::sim::TrainTrajectorySet::size() const {
  return trajectories.size();
}
