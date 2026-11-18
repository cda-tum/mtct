#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "plog/Log.h"
#include "simulator/GeneralSimulator.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "simulator/GreedySimulator.hpp"
#include "solver/astar-based/CompletionHeuristic.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iterator>
#include <map>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

using std::size_t;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,bugprone-unchecked-optional-access)

namespace {
// A train whose front leaves a vertex more than this after arriving there
// stands still at the vertex
constexpr double TIME_TOLERANCE = 1e-7;

// When the front of a train gets to a position on its route, or leaves it
// again, and its velocity then
struct FrontPassing {
  double t;
  double vel;
};

// The trajectory of a successful simulation starts when the train enters at
// position 0 and ends when its rear has left the network, beyond the end of its
// route. Hence, its front passes every position of the route. Between two
// samples, the train is assumed to accelerate constantly, as the simulator
// moves it.
FrontPassing
front_passing(const std::map<double, cda_rail::simulator::PosVel>& trajectory,
              double pos, bool leaving) {
  const auto next = std::ranges::find_if(trajectory, [pos, leaving](
                                                         const auto& entry) {
    return entry.second.pos > pos + cda_rail::GRB_EPS ||
           (!leaving && std::abs(entry.second.pos - pos) < cda_rail::GRB_EPS);
  });
  assert(next != trajectory.end());
  const auto& [t, sample] = *next;
  if (sample.pos <= pos + cda_rail::GRB_EPS) {
    return {.t = t, .vel = sample.vel};
  }
  const auto& [prev_t, prev_sample] = *std::prev(next);
  const auto distance               = std::max(pos - prev_sample.pos, 0.0);
  const auto share                  = distance / (sample.pos - prev_sample.pos);
  // The squared velocity is linear in the position, and the average velocity
  // is the mean of both velocities
  const auto vel =
      std::sqrt(std::max((prev_sample.vel * prev_sample.vel) +
                             (share * ((sample.vel * sample.vel) -
                                       (prev_sample.vel * prev_sample.vel))),
                         0.0));
  const auto mean_vel = (prev_sample.vel + vel) / 2;
  return {.t   = mean_vel > cda_rail::EPS ? prev_t + (distance / mean_vel)
                                          : prev_t + (share * (t - prev_t)),
          .vel = vel};
}

// The trains in the order of their times, ties broken by train index
cda_rail::index_vector
ordered_trains(std::vector<std::pair<double, size_t>> times) {
  std::ranges::sort(times);
  cda_rail::index_vector trains;
  trains.reserve(times.size());
  for (const auto tr : times | std::views::values) {
    trains.emplace_back(tr);
  }
  return trains;
}

// GRBVar::sameAs is not const
bool exists(GRBVar var) { return !var.sameAs(GRBVar()); }
} // namespace

std::optional<cda_rail::simulator::SimulatorState> cda_rail::solver::mip_based::
    GenPOMovingBlockMIPSolver::completed_initial_state() const {
  simulator::SimulatorState state{
      .train_edges    = std::vector<cda_rail::index_vector>(m_num_tr),
      .ttd_orders     = std::vector<cda_rail::index_vector>(m_num_ttd),
      .vertex_orders  = std::vector<cda_rail::index_vector>(m_num_vertices),
      .stop_positions = std::vector<std::vector<double>>(m_num_tr)};
  if (m_model_detail.fix_routes) {
    for (size_t tr = 0; tr < m_num_tr; tr++) {
      const auto& tr_name =
          m_instance.get_const_train_list().get_train(tr).get_name();
      if (!m_instance.get_const_routes().has_route(tr_name)) {
        continue;
      }
      state.train_edges.at(tr) =
          m_instance.get_const_routes().get_route(tr_name).get_edges();
      if (!astar_based::serve_stops_along_route(m_instance, state, tr)) {
        return std::nullopt;
      }
    }
  }
  simulator::GreedySimulator simulator(m_instance, m_ttd_sections);
  return astar_based::complete_state(simulator, std::move(state),
                                     m_solver_strategy.completion_dt,
                                     m_model_detail.allow_late_entry, true);
}

std::optional<std::vector<size_t>>
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::velocity_indices(
    size_t tr, const cda_rail::index_vector& route,
    const std::vector<VertexPassing>& passings) const {
  const auto& network  = m_instance.get_const_network();
  const auto& train    = m_instance.get_const_train_list().get_train(tr);
  const auto& schedule = m_instance.get_const_schedule(tr);
  const auto& y        = m_vars.at("y");

  // Whether the train can pass the vertex with this index of the route at the
  // given velocity
  const auto vertex_allows = [&](size_t i, double vel) {
    if (vel > 0 && passings.at(i).stands_still) {
      return false;
    }
    // From its exit vertex on, it has to reach its exit velocity
    return i + 1 < passings.size() ||
           possible_by_eom(vel, schedule.get_exit_velocity(),
                           train.get_acceleration(), train.get_deceleration(),
                           train.get_length());
  };

  // By vertex of the route and velocity index there, the least sum of squared
  // differences up to that vertex, and the velocity index at the previous
  // vertex that attains it
  std::vector<std::vector<double>> cost;
  std::vector<std::vector<size_t>> previous;
  for (size_t i = 0; i < passings.size(); i++) {
    const auto  v          = i == 0 ? network.get_edge(route.front()).source
                                    : network.get_edge(route.at(i - 1)).target;
    const auto& velocities = m_velocity_extensions.at(tr).at(v);
    cost.emplace_back(velocities.size(), INF);
    previous.emplace_back(velocities.size(), 0);
    for (size_t j = 0; j < velocities.size(); j++) {
      if (!vertex_allows(i, velocities.at(j))) {
        continue;
      }
      const auto difference = velocities.at(j) - passings.at(i).velocity;
      const auto deviation  = difference * difference;
      if (i == 0) {
        cost.at(i).at(j) = deviation;
        continue;
      }
      for (size_t k = 0; k < cost.at(i - 1).size(); k++) {
        if (cost.at(i - 1).at(k) >= INF ||
            !exists(y.at(tr, route.at(i - 1), k, j))) {
          continue;
        }
        if (const auto c = cost.at(i - 1).at(k) + deviation;
            c < cost.at(i).at(j)) {
          cost.at(i).at(j)     = c;
          previous.at(i).at(j) = k;
        }
      }
    }
  }

  // Every vertex has at least one velocity
  const auto best = std::ranges::min_element(cost.back());
  if (*best >= INF) {
    return std::nullopt;
  }
  std::vector<size_t> indices(passings.size());
  indices.back() = static_cast<size_t>(best - cost.back().begin());
  for (size_t i = passings.size() - 1; i > 0; i--) {
    indices.at(i - 1) = previous.at(i).at(indices.at(i));
  }
  return indices;
}

std::optional<
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::SimulatedSolution>
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::simulate_state(
    simulator::SimulatorState state) const {
  simulator::GreedySimulator simulator(m_instance, m_ttd_sections);
  simulator.set_simulator_state(state);
  auto results =
      simulator.simulate(m_solver_strategy.completion_dt,
                         m_model_detail.allow_late_entry, true, true, true);
  if (!results.success) {
    return std::nullopt;
  }
  // The simulator does not know the delay bounds of the model
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& stops = m_instance.get_const_schedule(tr).get_stops();
    bool        within_bounds =
        results.exit_times.at(tr) <= latest_exit_time(tr) + GRB_EPS;
    for (size_t stop = 0; stop < stops.size(); stop++) {
      within_bounds =
          within_bounds && results.stop_times.at(tr).at(stop) -
                                   stops.at(stop).get_service_time() <=
                               m_model_detail.max_station_delay + GRB_EPS;
    }
    if (!within_bounds) {
      PLOGD << "The simulated solution exceeds the delay bounds of the model";
      return std::nullopt;
    }
  }
  const auto obj = simulator::objective_val(m_instance, results.exit_times,
                                            results.stop_times);
  return SimulatedSolution{
      .state = std::move(state), .results = std::move(results), .obj = obj};
}

std::optional<
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::SimulatedSolution>
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::set_warm_start() {
  const auto state = completed_initial_state();
  if (!state.has_value()) {
    PLOGD << "No MIP start, since the completion of the initial A* state "
             "failed";
    return std::nullopt;
  }
  auto simulated = simulate_state(state.value());
  if (!simulated.has_value()) {
    return std::nullopt;
  }
  PLOGD << "Simulated the completed initial A* state with objective "
        << simulated->obj;
  if (const auto values = integer_values(simulated.value());
      values.has_value()) {
    PLOGD << "MIP start from the simulation of the completed initial A* state";
    for (auto [var, value] : values.value()) {
      var.set(GRB_DoubleAttr_Start, value);
    }
  }
  return simulated;
}

std::optional<std::vector<std::pair<GRBVar, double>>>
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::integer_values(
    const SimulatedSolution& solution) {
  const auto& state   = solution.state;
  const auto& results = solution.results;
  const auto& network = m_instance.get_const_network();

  // The integer variables by their index in the model, with their values. The
  // model has a variable for every route, stop and order of the completed
  // state, since the completion only uses edges on which a train can get from
  // its entry to its exit vertex.
  std::map<int, std::pair<GRBVar, double>> values;
  const auto set = [&values](const GRBVar& var, double value) {
    assert(exists(var));
    values.insert_or_assign(var.index(), std::pair{var, value});
  };
  // Every integer variable that is not set to 1 below is 0
  for (const auto* name : {"x", "y", "order", "x_ttd", "order_ttd", "stop",
                           "reverse_order", "waiting_at_vertex"}) {
    if (const auto it = m_vars.find(name); it != m_vars.end()) {
      for (const auto& var : std::as_const(it->second)) {
        if (exists(var)) {
          set(var, 0);
        }
      }
    }
  }

  // When the front of a train enters an edge of its route, by train and edge
  std::vector<std::map<size_t, double>> edge_entry_times(m_num_tr);
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& route      = state.train_edges.at(tr);
    const auto& trajectory = results.train_trajectories.at(tr);

    // How the train passes the vertices of its route, and where it stops
    cda_rail::index_vector     vertices{network.get_edge(route.front()).source};
    std::vector<VertexPassing> passings;
    double                     pos      = 0;
    size_t                     stop_idx = 0;
    for (size_t i = 0; i <= route.size(); i++) {
      if (i > 0) {
        vertices.emplace_back(network.get_edge(route.at(i - 1)).target);
        pos += network.get_edge(route.at(i - 1)).length;
      }
      const auto  arrival        = front_passing(trajectory, pos, false);
      const auto  departure      = front_passing(trajectory, pos, true);
      const auto& stop_positions = state.stop_positions.at(tr);
      const bool  stop = stop_idx < stop_positions.size() &&
                         std::abs(stop_positions.at(stop_idx) - pos) < GRB_EPS;
      if (stop) {
        set(m_vars["stop"](tr, stop_idx++, vertices.back()), 1);
      }
      // The model does not let a train stand still at its exit vertex, see
      // set_warm_start
      passings.push_back(
          {.departure    = departure.t,
           .velocity     = departure.vel,
           .stands_still = stop || (i < route.size() &&
                                    departure.t > arrival.t + TIME_TOLERANCE)});
    }

    for (size_t i = 0; i < route.size(); i++) {
      set(m_vars["x"](tr, route.at(i)), 1);
      if (const auto ttd =
              Network::get_ttd_of_edge(route.at(i), m_ttd_sections);
          ttd.has_value()) {
        set(m_vars["x_ttd"](tr, ttd.value()), 1);
      }
      edge_entry_times.at(tr)[route.at(i)] = passings.at(i).departure;
    }
    const auto indices = velocity_indices(tr, route, passings);
    if (!indices.has_value()) {
      PLOGD << "No MIP start, since the model has no velocities for how "
            << m_instance.get_const_train_list().get_train(tr).get_name()
            << " passes its route in the simulation";
      return std::nullopt;
    }
    for (size_t i = 0; i < route.size(); i++) {
      set(m_vars["y"](tr, route.at(i), indices->at(i), indices->at(i + 1)), 1);
    }
    if (m_vars.contains("waiting_at_vertex")) {
      for (size_t i = 0; i < vertices.size(); i++) {
        if (m_velocity_extensions.at(tr)
                .at(vertices.at(i))
                .at(indices->at(i)) == 0) {
          // A train may wait wherever it passes at velocity 0
          set(m_vars["waiting_at_vertex"](tr, vertices.at(i)), 1);
        }
      }
    }
  }

  // Of two trains, the one whose front enters later follows the other one
  const auto set_orders = [&set](MultiArray<GRBVar>& order, size_t index,
                                 std::vector<std::pair<double, size_t>> times) {
    const auto trains = ordered_trains(std::move(times));
    for (size_t i = 0; i < trains.size(); i++) {
      for (size_t j = i + 1; j < trains.size(); j++) {
        set(order(trains.at(j), trains.at(i), index), 1);
      }
    }
  };
  for (size_t e = 0; e < m_num_edges; e++) {
    std::vector<std::pair<double, size_t>> times;
    for (size_t tr = 0; tr < m_num_tr; tr++) {
      if (const auto it = edge_entry_times.at(tr).find(e);
          it != edge_entry_times.at(tr).end()) {
        times.emplace_back(it->second, tr);
      }
    }
    set_orders(m_vars["order"], e, std::move(times));
  }
  for (size_t ttd = 0; ttd < m_num_ttd; ttd++) {
    std::vector<std::pair<double, size_t>> times;
    for (size_t tr = 0; tr < m_num_tr; tr++) {
      std::optional<double> entry_time;
      for (const auto& [e, t] : edge_entry_times.at(tr)) {
        if (m_ttd_sections.at(ttd).contains(e)) {
          entry_time = std::min(entry_time.value_or(INF), t);
        }
      }
      if (entry_time.has_value()) {
        times.emplace_back(entry_time.value(), tr);
      }
    }
    set_orders(m_vars["order_ttd"], ttd, std::move(times));
  }
  // On a bidirectional track, only trains in opposite directions are ordered
  for (size_t idx = 0; idx < m_relevant_reverse_edges.size(); idx++) {
    const auto& [e1, e2] = m_relevant_reverse_edges.at(idx);
    std::vector<std::pair<double, size_t>> times_1;
    std::vector<std::pair<double, size_t>> times_2;
    for (size_t tr = 0; tr < m_num_tr; tr++) {
      if (const auto it = edge_entry_times.at(tr).find(e1);
          it != edge_entry_times.at(tr).end()) {
        times_1.emplace_back(it->second, tr);
      } else if (const auto it_2 = edge_entry_times.at(tr).find(e2);
                 it_2 != edge_entry_times.at(tr).end()) {
        times_2.emplace_back(it_2->second, tr);
      }
    }
    for (const auto& [t_1, tr_1] : times_1) {
      for (const auto& [t_2, tr_2] : times_2) {
        set(t_1 < t_2 ? m_vars["reverse_order"](tr_2, tr_1, idx)
                      : m_vars["reverse_order"](tr_1, tr_2, idx),
            1);
      }
    }
  }

  std::vector<std::pair<GRBVar, double>> result;
  result.reserve(values.size());
  for (const auto& [var, value] : values | std::views::values) {
    result.emplace_back(var, value);
  }
  return result;
}

cda_rail::simulator::SimulatorState
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::relaxation_state(
    const std::function<double(const GRBVar&)>& value) const {
  const auto&               network = m_instance.get_const_network();
  simulator::SimulatorState state{
      .train_edges    = std::vector<cda_rail::index_vector>(m_num_tr),
      .ttd_orders     = std::vector<cda_rail::index_vector>(m_num_ttd),
      .vertex_orders  = std::vector<cda_rail::index_vector>(m_num_vertices),
      .stop_positions = std::vector<std::vector<double>>(m_num_tr)};
  // The value of a variable, 0 if the model does not have it
  const auto value_of = [&value](const GRBVar& var) {
    return exists(var) ? value(var) : 0.0;
  };

  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& schedule = m_instance.get_const_schedule(tr);
    auto&       route    = state.train_edges.at(tr);
    // The candidate not yet on the route with the highest positive value,
    // ties broken by edge index, since the candidates are unordered
    const auto best_edge =
        [&](const cda_rail::index_set& candidates) -> std::optional<size_t> {
      cda_rail::index_vector sorted(candidates.begin(), candidates.end());
      std::ranges::sort(sorted);
      std::optional<size_t> best;
      double                best_value = DEFAULT_INT_FEAS_TOL;
      for (const auto e : sorted) {
        const auto val = value_of(m_vars.at("x").at(tr, e));
        if (!std::ranges::contains(route, e) && val > best_value) {
          best       = e;
          best_value = val;
        }
      }
      return best;
    };
    auto next = best_edge(network.out_edges(schedule.get_entry_vertex()));
    while (next.has_value()) {
      route.emplace_back(next.value());
      next = network.get_edge(next.value()).target == schedule.get_exit_vertex()
                 ? std::nullopt
                 : best_edge(network.get_successors(next.value()));
    }
    // Stops that cannot be served on the route are left to the completion
    astar_based::serve_stops_along_route(m_instance, state, tr);
  }

  return state;
}

std::optional<bool>
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::complete_relaxation(
    const std::function<double(const GRBVar&)>& value) {
  simulator::GreedySimulator simulator(m_instance, m_ttd_sections);
  auto                       completed = astar_based::complete_state(
      simulator, relaxation_state(value), m_solver_strategy.completion_dt,
      m_model_detail.allow_late_entry, true);
  if (!completed.has_value()) {
    return std::nullopt;
  }
  auto simulated = simulate_state(std::move(completed.value()));
  if (!simulated.has_value()) {
    return std::nullopt;
  }
  if (m_simulated_incumbent.has_value() &&
      simulated->obj >= m_simulated_incumbent->obj) {
    return false;
  }
  m_simulated_incumbent = std::move(simulated);
  return true;
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback::
    complete_node_relaxation() {
  const auto interval = solver->m_solver_strategy.completion_interval;
  // With fixed routes, a relaxation cannot change the routes, hence only the
  // initial state is completed. Gurobi does not stop within a callback, hence
  // no completion is started once the time limit is reached.
  if (interval == 0 || solver->m_model_detail.fix_routes ||
      getIntInfo(GRB_CB_MIPNODE_STATUS) != GRB_OPTIMAL ||
      getDoubleInfo(GRB_CB_RUNTIME) >=
          solver->m_model->get(GRB_DoubleParam_TimeLimit)) {
    return;
  }
  const auto relaxation = m_num_relaxations++;
  if (relaxation % interval != 0) {
    return;
  }
  m_completions_attempted++;
  const auto improved = solver->complete_relaxation(
      [this](const GRBVar& var) { return getNodeRel(var); });
  if (!improved.has_value()) {
    return;
  }
  m_completions_found++;
  if (!improved.value()) {
    return;
  }
  const auto& incumbent = solver->m_simulated_incumbent.value();
  PLOGD << "Completion of node relaxation " << relaxation + 1
        << " improves the simulated incumbent to " << incumbent.obj;
  // Gurobi only has to find solutions at least as good
  addLazy(solver->m_objective_expr <= incumbent.obj);
  if (const auto values = solver->integer_values(incumbent);
      values.has_value()) {
    std::vector<GRBVar> vars;
    std::vector<double> vals;
    for (const auto& [var, val] : values.value()) {
      vars.emplace_back(var);
      vals.emplace_back(val);
    }
    setSolution(vars.data(), vals.data(), static_cast<int>(vars.size()));
    useSolution();
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,bugprone-unchecked-optional-access)
