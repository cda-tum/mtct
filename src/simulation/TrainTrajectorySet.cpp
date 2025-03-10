#include "simulation/TrainTrajectorySet.hpp"

cda_rail::sim::TrainTrajectorySet::TrainTrajectorySet(
    const SimulationInstance& instance)
    : instance(instance) {}

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

void cda_rail::sim::TrainTrajectorySet::insert_or_assign(
    std::string name, cda_rail::sim::TrainTrajectory traj) {
  const TrainList& trainlist = instance.get().timetable.get_train_list();
  if (!trainlist.has_train(name))
    throw std::invalid_argument("No train with such name in timetable.");

  trajectories.insert_or_assign(name, traj);
}

std::optional<double> cda_rail::sim::TrainTrajectorySet::train_distance(
    std::string train1, std::string train2, size_t timestep) const {
  if (!contains(train1) || !contains(train2)) {
    return {};
  }

  const std::optional<TrainState> state_opt1 =
      trajectories.at(train1).get_state(timestep);
  const std::optional<TrainState> state_opt2 =
      trajectories.at(train2).get_state(timestep);

  if (!state_opt1.has_value() || !state_opt2.has_value()) {
    return {};
  }

  const TrainState state1 = state_opt1.value();
  const TrainState state2 = state_opt2.value();
  const Edge&      edge1  = instance.get().network.get_edge(state1.edge);
  const Edge&      edge2  = instance.get().network.get_edge(state2.edge);

  if (state1.edge == state2.edge)
    return std::abs(state1.position - state2.position) * edge1.length;

  const double remain_dist_1_fw = (1 - state1.position) * edge1.length;
  const double remain_dist_1_bw = state1.position * edge1.length;
  const double remain_dist_2_fw = (1 - state2.position) * edge2.length;
  const double remain_dist_2_bw = state2.position * edge2.length;

  const double dist_fw_fw =
      remain_dist_1_fw + remain_dist_2_fw +
      instance.get().shortest_paths[edge1.target][edge2.target];
  const double dist_fw_bw =
      remain_dist_1_fw + remain_dist_2_bw +
      instance.get().shortest_paths[edge1.target][edge2.source];
  const double dist_bw_bw =
      remain_dist_1_bw + remain_dist_2_bw +
      instance.get().shortest_paths[edge1.source][edge2.source];
  const double dist_bw_fw =
      remain_dist_1_bw + remain_dist_2_fw +
      instance.get().shortest_paths[edge1.source][edge2.target];

  const double min_dist = std::min(
      dist_fw_fw, std::min(dist_fw_bw, std::min(dist_bw_bw, dist_bw_fw)));

  if (min_dist < 0)
    throw std::logic_error("Distance calculation failed.");

  return min_dist;
}

std::optional<double> cda_rail::sim::TrainTrajectorySet::train_vertex_distance(
    std::string train, size_t vertex, size_t timestep) const {
  if (!contains(train))
    return {};

  const std::optional<TrainState> state_opt =
      trajectories.at(train).get_state(timestep);

  if (!state_opt.has_value())
    return {};

  const TrainState state = state_opt.value();
  const Edge&      edge  = instance.get().network.get_edge(state.edge);

  const double remain_dist_fw = (1 - state.position) * edge.length;
  const double remain_dist_bw = state.position * edge.length;

  if (edge.target == vertex)
    return remain_dist_fw;
  if (edge.source == vertex)
    return remain_dist_bw;

  const double dist_fw =
      remain_dist_fw + instance.get().shortest_paths[edge.target][vertex];
  const double dist_bw =
      remain_dist_bw + instance.get().shortest_paths[edge.source][vertex];

  const double min_dist = std::min(dist_fw, dist_bw);

  if (min_dist < 0)
    throw std::logic_error("Distance calculation failed.");

  return min_dist;
}

void cda_rail::sim::TrainTrajectorySet::export_csv(
    const std::filesystem::path& p) const {
  std::ofstream csvfile(p);
  csvfile << "train_idx,train_name,timestep,edge_idx,edge_src_node,edge_dst_"
             "node,edge_pos,speed\n";

  const TrainList& train_list = instance.get().timetable.get_train_list();

  for (auto traj : trajectories) {
    for (size_t timestep = traj.second.get_first_timestep();
         timestep <= traj.second.get_last_timestep(); timestep++) {
      TrainState  state = traj.second.get_state(timestep).value();
      const Edge& edge  = instance.get().network.get_edge(state.edge);

      csvfile << train_list.get_train_index(traj.first) << ",";
      csvfile << traj.first << ",";
      csvfile << timestep << ",";
      csvfile << state.edge << ",";
      csvfile << instance.get().network.get_vertex(edge.source).name << ",";
      csvfile << instance.get().network.get_vertex(edge.target).name << ",";
      csvfile << state.position << ",";
      csvfile << state.speed << "\n";
    }
  }

  csvfile.close();
}

void cda_rail::sim::TrainTrajectorySet::check_speed_limits() const {
  for (auto traj : trajectories) {
    traj.second.check_speed_limits();
  }
}

const std::unordered_map<std::string, cda_rail::sim::TrainTrajectory>&
cda_rail::sim::TrainTrajectorySet::get_map() const {
  return trajectories;
}

std::optional<cda_rail::sim::TrainTrajectory>
cda_rail::sim::TrainTrajectorySet::get_traj(std::string train_name) const {
  if (contains(train_name)) {
    return trajectories.at(train_name);
  } else {
    return {};
  }
}

const cda_rail::sim::SimulationInstance&
cda_rail::sim::TrainTrajectorySet::get_instance() const {
  return instance;
}

size_t cda_rail::sim::TrainTrajectorySet::size() const {
  return trajectories.size();
}

bool cda_rail::sim::TrainTrajectorySet::contains(std::string train_name) const {
  return trajectories.count(train_name);
}

cda_rail::instances::SolGeneralPerformanceOptimizationInstance<
    cda_rail::instances::GeneralPerformanceOptimizationInstance>
cda_rail::sim::TrainTrajectorySet::to_vss_solution(
    const cda_rail::Network& network_bidirec) const {
  cda_rail::instances::GeneralPerformanceOptimizationInstance general_instance{
      network_bidirec, instance.get().timetable.parse_to_general_timetable(),
      RouteMap{}};
  cda_rail::instances::SolGeneralPerformanceOptimizationInstance<
      cda_rail::instances::GeneralPerformanceOptimizationInstance>
      general_sol_instance{general_instance};

  for (auto [tr_name, traj] : trajectories) {
    Route                                  route;
    std::vector<std::pair<double, double>> route_positions;
    std::tie(route, route_positions) =
        traj.convert_to_vss_format(network_bidirec);

    for (std::pair<double, double> route_position : route_positions) {
      general_sol_instance.add_train_pos(tr_name, route_position.first,
                                         route_position.second);
      general_sol_instance.add_train_speed(
          tr_name, route_position.first,
          traj.get_state(route_position.first).value().speed);
    }

    general_sol_instance.set_route(tr_name, route);
    general_sol_instance.set_train_routed(tr_name);
  }

  general_sol_instance.check_consistency();
  return general_sol_instance;
}
