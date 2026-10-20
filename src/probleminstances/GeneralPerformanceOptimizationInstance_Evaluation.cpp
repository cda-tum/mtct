#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
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

// Trains ordered by the time of their event.
cda_rail::index_vector
sorted_trains(std::vector<std::pair<double, size_t>> events) {
  std::ranges::sort(events);
  cda_rail::index_vector trains;
  trains.reserve(events.size());
  std::ranges::copy(events | std::views::values, std::back_inserter(trains));
  return trains;
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
    if (train_edges.at(tr).empty()) {
      continue;
    }
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
      });
  const auto result = simulator.simulate(dt, true, true, false, true);

  SolutionEvaluation evaluation{.dt = dt, .success = result.success};
  if (result.success) {
    evaluation.obj =
        setup.instance.get_objective_val(result.exit_times, result.stop_times);
    evaluation.exit_times = result.exit_times;
    for (size_t tr = 0; tr < num_tr; ++tr) {
      const auto& schedule = setup.instance.get_const_schedule(tr);
      evaluation.exit_delays.emplace_back(result.exit_times.at(tr) -
                                          schedule.get_exit_time());
      // The simulator reports when a train reaches its stop, the service
      // starts no earlier than scheduled
      auto& tr_stop_times  = evaluation.stop_times.emplace_back();
      auto& tr_stop_delays = evaluation.stop_delays.emplace_back();
      for (size_t stop = 0; stop < schedule.get_stops().size(); ++stop) {
        const auto service_time =
            schedule.get_stops().at(stop).get_service_time();
        tr_stop_times.emplace_back(
            std::max(result.stop_times.at(tr).at(stop), service_time));
        tr_stop_delays.emplace_back(tr_stop_times.back() - service_time);
      }
    }
  }
  m_evaluation = std::move(evaluation);
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

  // NOLINTBEGIN(*-pro-bounds-avoid-unchecked-container-access)
  nlohmann::json evaluation;
  evaluation["dt"]      = m_evaluation->dt;
  evaluation["success"] = m_evaluation->success;
  if (m_evaluation->success) {
    evaluation["obj"]      = m_evaluation->obj;
    evaluation["trains"]   = nlohmann::json::object();
    auto const& train_list = get_instance()->get_const_train_list();
    for (size_t tr = 0; tr < train_list.size(); ++tr) {
      evaluation["trains"][train_list.get_train(tr).get_name()] = {
          {"exit_time", m_evaluation->exit_times.at(tr)},
          {"exit_delay", m_evaluation->exit_delays.at(tr)},
          {"stop_times", m_evaluation->stop_times.at(tr)},
          {"stop_delays", m_evaluation->stop_delays.at(tr)}};
    }
  }
  data["evaluation"] = evaluation;
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

  const auto&        evaluation_json = data.at("evaluation");
  SolutionEvaluation evaluation{.dt = evaluation_json.at("dt").get<double>(),
                                .success =
                                    evaluation_json.at("success").get<bool>()};
  if (evaluation.success) {
    evaluation.obj         = evaluation_json.at("obj").get<double>();
    auto const& train_list = get_instance()->get_const_train_list();
    for (size_t tr = 0; tr < train_list.size(); ++tr) {
      const auto& tr_json =
          evaluation_json.at("trains").at(train_list.get_train(tr).get_name());
      evaluation.exit_times.emplace_back(tr_json.at("exit_time").get<double>());
      evaluation.exit_delays.emplace_back(
          tr_json.at("exit_delay").get<double>());
      evaluation.stop_times.emplace_back(
          tr_json.at("stop_times").get<std::vector<double>>());
      evaluation.stop_delays.emplace_back(
          tr_json.at("stop_delays").get<std::vector<double>>());
    }
  }
  m_evaluation = std::move(evaluation);
}
