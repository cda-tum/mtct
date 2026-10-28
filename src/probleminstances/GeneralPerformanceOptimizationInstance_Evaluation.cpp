#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "datastructure/Train.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"
#include "simulator/GeneralSimulator.hpp"
#include "simulator/GreedySimulator.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <optional>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

namespace {
// Time at which the front of a train passes a route position in the solution,
// interpolated linearly between the last sample not beyond that position and
// the first one beyond it. A train standing at the position has not passed it.
double time_front_passes(
    const cda_rail::instances::SolGeneralPerformanceOptimizationInstance& sol,
    const std::string& tr_name, double pos) {
  std::optional<std::pair<double, double>> prev_sample;
  for (const auto t : sol.get_train_times(tr_name)) {
    const auto t_pos = sol.get_train_pos(tr_name, t);
    if (t_pos > pos + cda_rail::GRB_EPS) {
      if (!prev_sample.has_value()) {
        return t;
      }
      const auto [prev_t, prev_pos] = prev_sample.value();
      return prev_t + (std::max(pos - prev_pos, 0.0) / (t_pos - prev_pos) *
                       (t - prev_t));
    }
    prev_sample = {t, t_pos};
  }
  throw cda_rail::exceptions::ConsistencyException(
      "Train " + tr_name + " never passes position " + std::to_string(pos));
}

// Position of a train in the solution at its first sample not before t.
double pos_at_first_sample_from(
    const cda_rail::instances::SolGeneralPerformanceOptimizationInstance& sol,
    const std::string& tr_name, double t) {
  for (const auto sample_t : sol.get_train_times(tr_name)) {
    if (sample_t + cda_rail::GRB_EPS >= t) {
      return sol.get_train_pos(tr_name, sample_t);
    }
  }
  throw cda_rail::exceptions::ConsistencyException(
      "Train " + tr_name + " has no position after time " + std::to_string(t));
}

// Whether the trajectory of a train in the solution follows its dynamics,
// i.e., whether the train can get from every sample to the next one with its
// acceleration and deceleration. The positions may be off by the precision to
// which the solver rounded them. Kept behind such a trajectory, the train can
// always stop where the train of the solution can.
bool trajectory_follows_dynamics(
    const cda_rail::instances::SolGeneralPerformanceOptimizationInstance& sol,
    const cda_rail::Train& train) {
  const auto& tr_name = train.get_name();
  const auto  times   = sol.get_train_times(tr_name);
  for (size_t i = 1; i < times.size(); ++i) {
    const auto t_1 = times.at(i - 1);
    const auto t_2 = times.at(i);
    if (!cda_rail::possible_by_eom(
            sol.get_train_speed(tr_name, t_1),
            sol.get_train_speed(tr_name, t_2), train.get_acceleration(),
            train.get_deceleration(),
            sol.get_train_pos(tr_name, t_2) - sol.get_train_pos(tr_name, t_1) +
                cda_rail::ROUNDING_PRECISION)) {
      return false;
    }
  }
  return true;
}

// Trains ordered by the time of their event.
cda_rail::index_vector
sorted_trains(std::vector<std::pair<double, size_t>> events) {
  std::ranges::sort(events);
  cda_rail::index_vector trains;
  trains.reserve(events.size());
  std::ranges::copy(events | std::views::values, std::back_inserter(trains));
  return trains;
}

// The delays of the trains in one simulation of the decisions of a solution.
cda_rail::instances::SimulatedDelays simulated_delays(
    const cda_rail::instances::GeneralPerformanceOptimizationInstance& instance,
    const cda_rail::simulator::SimulatorResults&                       result) {
  cda_rail::instances::SimulatedDelays delays{.success = result.success};
  if (!result.success) {
    return delays;
  }
  delays.obj = instance.get_objective_val(result.exit_times, result.stop_times);
  delays.exit_times = result.exit_times;
  for (size_t tr = 0; tr < instance.get_const_train_list().size(); ++tr) {
    const auto& schedule = instance.get_const_schedule(tr);
    delays.exit_delays.emplace_back(result.exit_times.at(tr) -
                                    schedule.get_exit_time());
    // The simulator reports when a train reaches its stop, the service starts
    // no earlier than scheduled
    auto& tr_stop_times  = delays.stop_times.emplace_back();
    auto& tr_stop_delays = delays.stop_delays.emplace_back();
    for (size_t stop = 0; stop < schedule.get_stops().size(); ++stop) {
      const auto service_time =
          schedule.get_stops().at(stop).get_service_time();
      tr_stop_times.emplace_back(
          std::max(result.stop_times.at(tr).at(stop), service_time));
      tr_stop_delays.emplace_back(tr_stop_times.back() - service_time);
    }
  }
  return delays;
}

// NOLINTBEGIN(*-pro-bounds-avoid-unchecked-container-access)
nlohmann::json
simulated_delays_to_json(const cda_rail::instances::SimulatedDelays& delays,
                         const cda_rail::TrainList& train_list) {
  nlohmann::json json;
  json["success"] = delays.success;
  if (!delays.success) {
    return json;
  }
  json["obj"]    = delays.obj;
  json["trains"] = nlohmann::json::object();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    json["trains"][train_list.get_train(tr).get_name()] = {
        {"exit_time", delays.exit_times.at(tr)},
        {"exit_delay", delays.exit_delays.at(tr)},
        {"stop_times", delays.stop_times.at(tr)},
        {"stop_delays", delays.stop_delays.at(tr)}};
  }
  return json;
}
// NOLINTEND(*-pro-bounds-avoid-unchecked-container-access)

cda_rail::instances::SimulatedDelays
simulated_delays_from_json(const nlohmann::json&      json,
                           const cda_rail::TrainList& train_list) {
  cda_rail::instances::SimulatedDelays delays{
      .success = json.at("success").get<bool>()};
  if (!delays.success) {
    return delays;
  }
  delays.obj = json.at("obj").get<double>();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    const auto& tr_json =
        json.at("trains").at(train_list.get_train(tr).get_name());
    delays.exit_times.emplace_back(tr_json.at("exit_time").get<double>());
    delays.exit_delays.emplace_back(tr_json.at("exit_delay").get<double>());
    delays.stop_times.emplace_back(
        tr_json.at("stop_times").get<std::vector<double>>());
    delays.stop_delays.emplace_back(
        tr_json.at("stop_delays").get<std::vector<double>>());
  }
  return delays;
}
} // namespace

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::evaluate(
    double dt) {
  exceptions::throw_if_non_positive(dt, "Time step length dt");
  if (!has_solution()) {
    throw exceptions::ConsistencyException("There is no solution to evaluate.");
  }

  const auto  setup      = get_evaluation_setup();
  const auto& network    = setup.instance.get_const_network();
  const auto& train_list = setup.instance.get_const_train_list();
  const auto  num_tr     = train_list.size();

  std::vector<cda_rail::index_vector> train_edges;
  train_edges.reserve(num_tr);
  for (size_t tr = 0; tr < num_tr; ++tr) {
    train_edges.emplace_back(
        setup.routes.get_route(train_list.get_train(tr).get_name())
            .get_edges());
  }

  // Every block section is passed in the order in which the fronts of the
  // trains enter it in this solution
  std::vector<cda_rail::index_vector> ttd_orders;
  ttd_orders.reserve(setup.ttd_sections.size());
  for (const auto& section : setup.ttd_sections) {
    std::vector<std::pair<double, size_t>> section_entries;
    for (size_t tr = 0; tr < num_tr; ++tr) {
      double pos = 0;
      for (const auto edge : train_edges.at(tr)) {
        if (section.contains(edge)) {
          section_entries.emplace_back(
              time_front_passes(*this, train_list.get_train(tr).get_name(),
                                pos),
              tr);
          break;
        }
        pos += network.get_edge(edge).length;
      }
    }
    ttd_orders.emplace_back(sorted_trains(std::move(section_entries)));
  }

  // Every entry and exit vertex is passed in the order in which the trains
  // enter or leave the network there in this solution
  std::vector<std::vector<std::pair<double, size_t>>> vertex_events(
      network.number_of_vertices());
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name  = train_list.get_train(tr).get_name();
    const auto& schedule = setup.instance.get_const_schedule(tr);
    vertex_events.at(schedule.get_entry_vertex())
        .emplace_back(time_front_passes(*this, tr_name, 0), tr);
    vertex_events.at(schedule.get_exit_vertex())
        .emplace_back(get_exit_time(tr_name), tr);
  }
  std::vector<cda_rail::index_vector> vertex_orders;
  vertex_orders.reserve(vertex_events.size());
  for (auto& events : vertex_events) {
    vertex_orders.emplace_back(sorted_trains(std::move(events)));
  }

  // Every train stops where it stands in this solution when its service starts
  std::vector<std::vector<double>> stop_positions(num_tr);
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name = train_list.get_train(tr).get_name();
    for (const auto stop_time : get_stop_times(tr_name)) {
      stop_positions.at(tr).emplace_back(
          pos_at_first_sample_from(*this, tr_name, stop_time));
    }
  }

  // A train whose trajectory follows its dynamics can always be kept behind
  // it, so that it leaves no earlier than in this solution
  cda_rail::index_set trains_following_dynamics;
  for (size_t tr = 0; tr < num_tr; ++tr) {
    if (trajectory_follows_dynamics(*this, train_list.get_train(tr))) {
      trains_following_dynamics.insert(tr);
    }
  }

  simulator::GreedySimulator simulator(
      setup.instance, setup.ttd_sections, std::move(train_edges),
      std::move(ttd_orders), std::move(vertex_orders),
      std::move(stop_positions));
  simulator.set_position_bound(
      [this](size_t tr, double t)
          -> std::optional<simulator::GreedySimulator::PositionBound> {
        const auto& train =
            get_instance()->get_const_train_list().get_train(tr);
        const auto& tr_name = train.get_name();
        const auto  times   = get_train_times(tr_name);
        if (times.empty() || t > times.back() + GRB_EPS) {
          // The train of this solution has left the network
          return std::nullopt;
        }
        if (t < times.front() - GRB_EPS) {
          // The train of this solution has not entered the network yet
          return simulator::GreedySimulator::PositionBound{
              .front = -INF, .braking_point = -INF};
        }
        const auto t_sample = std::clamp(t, times.front(), times.back());

        // The train is known exactly at the next sample. To have its speed
        // there, it has to cover at least the following distance until then,
        // since its acceleration is limited. Its braking point never decreases.
        [[maybe_unused]] const auto [edge, prev_t, next_t] =
            get_edge_and_time_bounds(tr_name, t_sample);
        const auto next_pos = get_train_pos(tr_name, next_t);
        const auto next_vel = get_train_speed(tr_name, next_t);
        const auto a        = train.get_acceleration();
        const auto tau      = next_t - t_sample;
        const auto min_dist = next_vel <= a * tau
                                  ? (next_vel * next_vel) / (2 * a)
                                  : (next_vel * tau) - (a * tau * tau / 2);
        simulator::GreedySimulator::PositionBound bound{
            .front = next_pos - min_dist,
            .braking_point =
                next_pos +
                braking_distance(next_vel, train.get_deceleration())};

        // In between, the trajectory can only be approximated. This is not
        // possible if the two samples contradict the train's dynamics, which
        // happens if the solver rounded the positions.
        if (!possible_by_eom(get_train_speed(tr_name, prev_t), next_vel, a,
                             train.get_deceleration(),
                             next_pos - get_train_pos(tr_name, prev_t))) {
          return bound;
        }
        if (const auto pos_vel =
                get_approximate_train_pos_and_vel(tr_name, t_sample);
            pos_vel.has_value()) {
          bound.front         = std::min(bound.front, pos_vel->pos);
          bound.braking_point = std::min(
              bound.braking_point,
              pos_vel->pos + braking_distance(std::max(pos_vel->vel, 0.0),
                                              train.get_deceleration()));
        }
        return bound;
      },
      std::move(trains_following_dynamics));
  const auto trajectory_result =
      simulator.simulate(dt, true, true, false, true);
  // Afterwards, only the decisions of this solution are fixed, as in the A*
  // search
  simulator.set_position_bound({});
  const auto greedy_result = simulator.simulate(dt, true, true, false, true);

  m_evaluation = SolutionEvaluation{
      .dt         = dt,
      .trajectory = simulated_delays(setup.instance, trajectory_result),
      .greedy     = simulated_delays(setup.instance, greedy_result)};
}

cda_rail::instances::SolGeneralPerformanceOptimizationInstance::EvaluationSetup
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    get_evaluation_setup() const {
  return {.instance = *get_instance(),
          .ttd_sections =
              get_instance()->get_const_network().unbreakable_sections(),
          .routes = get_const_solution_routes()};
}

cda_rail::instances::SolGeneralPerformanceOptimizationInstance::EvaluationSetup
cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    get_evaluation_setup() const {
  for (const auto& train : get_instance()->get_const_train_list()) {
    if (!train.has_tim()) {
      throw exceptions::InvalidInputException(
          "Evaluating trains without train integrity monitoring is not "
          "implemented for VSS solutions.");
    }
  }

  EvaluationSetup setup{.instance     = *get_instance(),
                        .ttd_sections = {},
                        .routes       = get_const_solution_routes()};
  setup.routes.update_after_discretization(
      setup.instance.separate_edges_at_vss(m_vss_pos));
  // Without moving block, every edge belongs to a block section
  auto& network = setup.instance.get_editable_network();
  for (size_t edge = 0; edge < network.number_of_edges(); ++edge) {
    network.set_edge_unbreakable(edge);
  }
  setup.ttd_sections = network.unbreakable_sections();
  return setup;
}

nlohmann::json cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    get_general_solution_data() const {
  auto data = SolGeneralProblemInstanceWithScheduleAndRoutes::
      get_general_solution_data();
  if (!m_evaluation.has_value()) {
    return data;
  }

  auto const& train_list = get_instance()->get_const_train_list();
  // NOLINTBEGIN(*-pro-bounds-avoid-unchecked-container-access)
  data["evaluation"] = {
      {"dt", m_evaluation->dt},
      {"trajectory",
       simulated_delays_to_json(m_evaluation->trajectory, train_list)},
      {"greedy", simulated_delays_to_json(m_evaluation->greedy, train_list)}};
  // NOLINTEND(*-pro-bounds-avoid-unchecked-container-access)
  return data;
}

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    set_general_solution_data(const nlohmann::json& data) {
  SolGeneralProblemInstanceWithScheduleAndRoutes::set_general_solution_data(
      data);
  m_evaluation.reset();
  if (!data.contains("evaluation")) {
    return;
  }

  const auto& evaluation_json = data.at("evaluation");
  auto const& train_list      = get_instance()->get_const_train_list();
  m_evaluation                = SolutionEvaluation{
      .dt         = evaluation_json.at("dt").get<double>(),
      .trajectory = simulated_delays_from_json(evaluation_json.at("trajectory"),
                                               train_list),
      .greedy =
          simulated_delays_from_json(evaluation_json.at("greedy"), train_list)};
}
