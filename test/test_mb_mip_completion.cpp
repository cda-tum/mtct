#define TEST_FRIENDS true

#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include "gtest/gtest.h"
#include <cmath>
#include <functional>
#include <optional>
#include <ranges>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// Tests of the completions of the moving block MIP, see
// SolverStrategyMovingBlock::completion_interval

#define EXPECT_APPROX_EQ(a, b)                                                 \
  EXPECT_TRUE(std::abs((a) - (b)) < 1e-2) << (a) << " !=(approx.) " << (b)

// NOLINTBEGIN (clang-analyzer-deadcode.DeadStores)

TEST(GenPOMovingBlockMIPSolver, WarmStartIsFeasible) {
  // The warm start sets every integer variable of the model, and only those,
  // but the helper binaries of indicator constraints, which are not part of
  // m_vars. Fixing them leaves a feasible model, including the lazy
  // constraints, i.e., Gurobi finds the times and these helpers. On the
  // smaller instances with every solver strategy that changes the integer
  // variables.
  using cda_rail::solver::mip_based::ModelDetailMovingBlock;
  using cda_rail::solver::mip_based::SolverStrategyMovingBlock;
  const std::vector<std::pair<std::string, std::string>> instances{
      {"HighSpeedTrack2Trains", "atmos2023"},
      {"HighSpeedTrack5Trains", "atmos2023"},
      {"SimpleStation", "atmos2023"},
      {"SingleTrack", "atmos2023"},
      {"SingleTrackWithStation", "atmos2023"},
      {"Overtake", "atmos2023"},
      {"SingleTrackCorridor", "atmos2023"},
      {"Stammstrecke4Trains", "atmos2023"},
      {"Stammstrecke8Trains", "atmos2023"},
      {"GeneralStammstrecke6Trains", "gen-po"}};
  for (const auto& [name, subdirectory] : instances) {
    const cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
        name, subdirectory, "./data");
    std::vector<ModelDetailMovingBlock> model_details{{}};
    if (instance.has_route_for_every_train()) {
      model_details.push_back({.fix_routes = true});
    }
    for (const auto& model_detail : model_details) {
      std::vector<SolverStrategyMovingBlock> strategies{{}};
      if (instance.get_const_train_list().size() <= 5) {
        strategies.push_back({.use_lazy_constraints = false});
        strategies.push_back({.use_indicator_constraints = true});
      }
      for (const auto& strategy : strategies) {
        const auto description =
            name + (model_detail.fix_routes ? " with fixed routes" : "") +
            (strategy.use_lazy_constraints ? "" : " without lazy") +
            (strategy.use_indicator_constraints ? " with indicators" : "");

        cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
        cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback
            callback(&solver);
        if (strategy.use_lazy_constraints) {
          solver.solve_init_general_mip(false, true, &callback);
          solver.m_model->set(GRB_IntParam_LazyConstraints, 1);
        } else {
          solver.solve_init_general_mip(false, true);
        }
        solver.m_instance.discretize_stops();
        solver.create_model({}, strategy, model_detail);
        solver.set_warm_start();
        solver.m_model->update();

        std::unordered_set<int> m_vars_indices;
        for (const auto& array : solver.m_vars | std::views::values) {
          for (auto var : array) {
            if (!var.sameAs(GRBVar())) {
              m_vars_indices.insert(var.index());
            }
          }
        }
        size_t                   start_vars = 0;
        std::vector<std::string> integers_without_start;
        for (int i = 0; i < solver.m_model->get(GRB_IntAttr_NumVars); ++i) {
          auto       var     = solver.m_model->getVar(i);
          const bool integer = var.get(GRB_CharAttr_VType) != GRB_CONTINUOUS;
          const auto start   = var.get(GRB_DoubleAttr_Start);
          if (start != GRB_UNDEFINED) {
            start_vars++;
            EXPECT_TRUE(integer)
                << description << ": " << var.get(GRB_StringAttr_VarName);
            var.set(GRB_DoubleAttr_LB, start);
            var.set(GRB_DoubleAttr_UB, start);
          } else if (integer && (!strategy.use_indicator_constraints ||
                                 m_vars_indices.contains(var.index()))) {
            integers_without_start.emplace_back(
                var.get(GRB_StringAttr_VarName));
          }
        }
        if (start_vars == 0) {
          // As in the A* search, the trains cannot follow their quickest
          // paths first come, first served, since these meet on the same
          // track of the passing loop
          EXPECT_TRUE(name == "SingleTrackCorridor" && !model_detail.fix_routes)
              << description;
        } else {
          EXPECT_TRUE(integers_without_start.empty())
              << description << ": " << integers_without_start.front();
          solver.m_model->set(GRB_IntParam_SolutionLimit, 1);
          solver.m_model->optimize();
          EXPECT_GT(solver.m_model->get(GRB_IntAttr_SolCount), 0)
              << description;
        }
        solver.cleanup();
      }
    }
  }
}

namespace {
// Trains Tr1 and Tr2 enter at v0 and leave at v5. Between v1 and v4, they take
// either the short route via v2 along the track of station S or the long one
// via v3.
struct TwoRoutes {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  cda_rail::index_vector                                      short_route;
  cda_rail::index_vector                                      long_route;
};

TwoRoutes two_routes() {
  TwoRoutes  routes;
  auto&      network = routes.instance.get_editable_network();
  const auto v0      = network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1      = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2      = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3      = network.add_vertex("v3", cda_rail::VertexType::TTD);
  const auto v4      = network.add_vertex("v4", cda_rail::VertexType::TTD);
  const auto v5      = network.add_vertex("v5", cda_rail::VertexType::TTD);

  const auto e01 = network.add_edge(v0, v1, 100, 10);
  const auto e12 = network.add_edge(v1, v2, 100, 10);
  const auto e24 = network.add_edge(v2, v4, 100, 10);
  const auto e13 = network.add_edge(v1, v3, 300, 10);
  const auto e34 = network.add_edge(v3, v4, 300, 10);
  const auto e45 = network.add_edge(v4, v5, 100, 10);
  network.add_successor(e01, e12);
  network.add_successor(e01, e13);
  network.add_successor(e12, e24);
  network.add_successor(e13, e34);
  network.add_successor(e24, e45);
  network.add_successor(e34, e45);
  routes.short_route = {e01, e12, e24, e45};
  routes.long_route  = {e01, e13, e34, e45};

  routes.instance.add_train("Tr1", 10, 10, 2, 2, true, 0, 10, v0, 0, 10, v5, 1);
  routes.instance.add_train("Tr2", 10, 10, 2, 2, true, 60, 10, v0, 60, 10, v5,
                            1);
  routes.instance.add_empty_station("S");
  routes.instance.add_track_to_station("S", e12);
  return routes;
}
} // namespace

TEST(GenPOMovingBlockMIPSolver, WarmStartRoutesTrainWithoutRoute) {
  // With fixed routes, Tr1 keeps its long route. Tr2 has no route, hence it
  // can take any route in the model, and the completion gives it the short one.
  auto routes = two_routes();
  routes.instance.add_empty_route("Tr1");
  for (const auto e : routes.long_route) {
    routes.instance.push_back_edge_to_route("Tr1", e);
  }

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {.fix_routes = true});
  const auto completed = solver.completed_initial_state();

  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->train_edges.at(0), routes.long_route);
  EXPECT_EQ(completed->train_edges.at(1), routes.short_route);
}

TEST(GenPOMovingBlockMIPSolver, NoWarmStartIfFixedRouteMissesStop) {
  // Tr1 is to stop at S, but its fixed route takes the long way around S
  auto routes = two_routes();
  routes.instance.insert_stop("Tr1", "S", 0, 30);
  routes.instance.add_empty_route("Tr1");
  for (const auto e : routes.long_route) {
    routes.instance.push_back_edge_to_route("Tr1", e);
  }

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {.fix_routes = true});

  EXPECT_FALSE(solver.completed_initial_state().has_value());
}

TEST(GenPOMovingBlockMIPSolver, NoWarmStartWithoutModelVelocities) {
  // Tr1 stops at v1, only 10 m before its exit vertex v2. Accelerating by
  // 2 m/s^2 from there, it cannot reach its exit velocity of 10 m/s before its
  // rear, 10 m behind its front, has left the network. The simulation lets it
  // leave slower, but the model has no velocities for this.
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  auto&      network = instance.get_editable_network();
  const auto v0      = network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1      = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2      = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto e01     = network.add_edge(v0, v1, 100, 10);
  const auto e12     = network.add_edge(v1, v2, 10, 10);
  network.add_successor(e01, e12);
  instance.add_train("Tr1", 10, 10, 2, 2, true, 0, 10, v0, 0, 10, v2, 1);
  instance.add_empty_station("S");
  instance.add_track_to_station("S", e01);
  instance.insert_stop("Tr1", "S", 0, 30);

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {});
  ASSERT_TRUE(solver.completed_initial_state().has_value());
  solver.set_warm_start();
  solver.m_model->update();

  EXPECT_EQ(solver.m_vars["x"](0, e01).get(GRB_DoubleAttr_Start),
            GRB_UNDEFINED);
}

TEST(GenPOMovingBlockMIPSolver, WarmStartKeepsOptimum) {
  // solve() finds the same optimum with the completions as without them
  const auto routes = two_routes();

  // with and without lazy constraints
  for (const bool use_lazy : {true, false}) {
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver_with(
        routes.instance);
    const auto sol_with = solver_with.solve({},
                                            {.use_lazy_constraints = use_lazy,
                                             .abs_mip_gap          = 1e-4,
                                             .completion_interval  = 1},
                                            {}, 60);
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver_without(
        routes.instance);
    const auto sol_without = solver_without.solve(
        {}, {.use_lazy_constraints = use_lazy, .abs_mip_gap = 1e-4}, {}, 60);

    ASSERT_TRUE(sol_with.has_solution()) << "use_lazy = " << use_lazy;
    ASSERT_TRUE(sol_without.has_solution()) << "use_lazy = " << use_lazy;
    EXPECT_EQ(sol_with.get_status(), cda_rail::SolutionStatus::Optimal);
    EXPECT_EQ(sol_without.get_status(), cda_rail::SolutionStatus::Optimal);
    EXPECT_APPROX_EQ(sol_with.get_obj(), sol_without.get_obj());
  }
}

namespace {
// The given relaxation values, 0 for every other variable
std::function<double(const GRBVar&)>
relaxation(std::vector<std::pair<GRBVar, double>> values) {
  return [values = std::move(values)](const GRBVar& var) {
    for (auto [v, val] : values) {
      if (v.sameAs(var)) {
        return val;
      }
    }
    return 0.0;
  };
}
} // namespace

TEST(GenPOMovingBlockMIPSolver, RelaxationStateFollowsHighestEdges) {
  // Tr1 takes the long route, whose edges have the higher values. No edge
  // after the first one of Tr2 has a positive value, hence its route ends
  // there, and the completion routes the rest.
  const auto  routes = two_routes();
  const auto& s      = routes.short_route;
  const auto& l      = routes.long_route;

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {});
  auto&      x     = solver.m_vars["x"];
  const auto state = solver.relaxation_state(relaxation({{x(0, l.at(0)), 1},
                                                         {x(0, s.at(1)), 0.4},
                                                         {x(0, s.at(2)), 0.4},
                                                         {x(0, l.at(1)), 0.6},
                                                         {x(0, l.at(2)), 0.6},
                                                         {x(0, l.at(3)), 1},
                                                         {x(1, l.at(0)), 1}}));

  EXPECT_EQ(state.train_edges.at(0), l);
  EXPECT_EQ(state.train_edges.at(1), cda_rail::index_vector{l.front()});
  EXPECT_TRUE(state.stop_positions.at(0).empty());
  EXPECT_TRUE(state.stop_positions.at(1).empty());
}

TEST(GenPOMovingBlockMIPSolver, RelaxationStateServesStopsOnRoute) {
  // Both trains are to stop at S. Tr1 takes the short route along S and stops
  // at its end, 200 m into the route. Tr2 takes the long route around S,
  // hence its route is emptied, and the completion routes it to S.
  auto routes = two_routes();
  routes.instance.insert_stop("Tr1", "S", 0, 30);
  routes.instance.insert_stop("Tr2", "S", 60, 90);
  const auto& s = routes.short_route;
  const auto& l = routes.long_route;

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {});
  std::vector<std::pair<GRBVar, double>> values;
  for (const auto e : s) {
    values.emplace_back(solver.m_vars["x"](0, e), 1);
  }
  for (const auto e : l) {
    values.emplace_back(solver.m_vars["x"](1, e), 1);
  }
  const auto state = solver.relaxation_state(relaxation(values));

  EXPECT_EQ(state.train_edges.at(0), s);
  EXPECT_EQ(state.stop_positions.at(0), std::vector<double>{200});
  EXPECT_TRUE(state.train_edges.at(1).empty());
  EXPECT_TRUE(state.stop_positions.at(1).empty());
}

TEST(GenPOMovingBlockMIPSolver, SimulatedIncumbentIfModelInfeasible) {
  // Tr1 stops at v1, only 10 m before its exit vertex v2, from where it
  // cannot reach its exit velocity of 10 m/s. The model is hence infeasible.
  // The simulation lets it leave slower, and this solution is returned. No
  // solution of the model is better, hence it is optimal. Its route replaces
  // the one of the instance.
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  auto&      network = instance.get_editable_network();
  const auto v0      = network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1      = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2      = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto e01     = network.add_edge(v0, v1, 100, 10);
  const auto e12     = network.add_edge(v1, v2, 10, 10);
  network.add_successor(e01, e12);
  instance.add_train("Tr1", 10, 10, 2, 2, true, 0, 10, v0, 0, 10, v2, 1);
  instance.add_empty_station("S");
  instance.add_track_to_station("S", e01);
  instance.insert_stop("Tr1", "S", 0, 30);
  instance.add_empty_route("Tr1");
  instance.push_back_edge_to_route("Tr1", e01);
  instance.push_back_edge_to_route("Tr1", e12);

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver_without(
      instance);
  const auto sol_without = solver_without.solve({}, {}, {}, 60);
  EXPECT_EQ(sol_without.get_status(), cda_rail::SolutionStatus::Infeasible);

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver_with(instance);
  const auto                                             sol_with =
      solver_with.solve({}, {.completion_interval = 1}, {}, 60);
  ASSERT_TRUE(sol_with.has_solution());
  EXPECT_EQ(sol_with.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_GT(sol_with.get_obj(), 0);
  EXPECT_EQ(sol_with.get_lower_bound(), sol_with.get_obj());
  EXPECT_EQ(sol_with.get_const_solution_routes().get_route("Tr1").get_edges(),
            (cda_rail::index_vector{e01, e12}));

  // The simulated solution delays the exit, which a maximal exit delay of 0
  // forbids
  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver_bounded(
      instance);
  const auto sol_bounded = solver_bounded.solve(
      {.max_exit_delay = 0}, {.completion_interval = 1}, {}, 60);
  EXPECT_FALSE(sol_bounded.has_solution());

  // It also starts serving its stop late, which a maximal station delay of 0
  // forbids
  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver_station(
      instance);
  const auto sol_station = solver_station.solve(
      {.max_station_delay = 0}, {.completion_interval = 1}, {}, 60);
  EXPECT_FALSE(sol_station.has_solution());
}

TEST(GenPOMovingBlockMIPSolver, RelaxationStateBreaksTiesByEdgeIndex) {
  // Both routes of Tr1 have the same values, hence it takes the edge with the
  // smaller index, which is the one of the short route
  const auto  routes = two_routes();
  const auto& s      = routes.short_route;
  const auto& l      = routes.long_route;
  ASSERT_LT(s.at(1), l.at(1));

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {});
  auto&      x     = solver.m_vars["x"];
  const auto state = solver.relaxation_state(relaxation({{x(0, s.at(0)), 1},
                                                         {x(0, s.at(1)), 0.5},
                                                         {x(0, s.at(2)), 0.5},
                                                         {x(0, l.at(1)), 0.5},
                                                         {x(0, l.at(2)), 0.5},
                                                         {x(0, s.at(3)), 1}}));

  EXPECT_EQ(state.train_edges.at(0), s);
}

TEST(GenPOMovingBlockMIPSolver, RelaxationStateSkipsEdgesOnRoute) {
  // After v0-v1-v2, Tr1 rather turns back to v1 than continuing to its exit
  // v3. From there, it could only use v1-v2 again, which is on its route
  // already, hence its route ends at v1, and the completion routes the rest.
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  auto&      network = instance.get_editable_network();
  const auto v0      = network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1      = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2      = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3      = network.add_vertex("v3", cda_rail::VertexType::TTD);
  const auto e01     = network.add_edge(v0, v1, 100, 10);
  const auto e12     = network.add_edge(v1, v2, 100, 10);
  const auto e21     = network.add_edge(v2, v1, 100, 10);
  const auto e23     = network.add_edge(v2, v3, 100, 10);
  network.add_successor(e01, e12);
  network.add_successor(e12, e21);
  network.add_successor(e21, e12);
  network.add_successor(e12, e23);
  instance.add_train("Tr1", 10, 10, 2, 2, true, 0, 10, v0, 0, 10, v3, 1);

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {});
  auto& x = solver.m_vars["x"];
  ASSERT_FALSE(x(0, e21).sameAs(GRBVar()));
  const auto state = solver.relaxation_state(relaxation(
      {{x(0, e01), 1}, {x(0, e12), 1}, {x(0, e21), 0.6}, {x(0, e23), 0.4}}));

  EXPECT_EQ(state.train_edges.at(0), (cda_rail::index_vector{e01, e12, e21}));
}

TEST(GenPOMovingBlockMIPSolver, RelaxationStateServesStopWithHighestValue) {
  // Station S consists of v1-v2 and v2-v3, hence Tr1 can stop at v2 or v3,
  // 200 m or 300 m into its route. It stops where the relaxation value is
  // higher.
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  auto&      network = instance.get_editable_network();
  const auto v0      = network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1      = network.add_vertex("v1", cda_rail::VertexType::TTD);
  const auto v2      = network.add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3      = network.add_vertex("v3", cda_rail::VertexType::TTD);
  const auto v4      = network.add_vertex("v4", cda_rail::VertexType::TTD);
  const auto e01     = network.add_edge(v0, v1, 100, 10);
  const auto e12     = network.add_edge(v1, v2, 100, 10);
  const auto e23     = network.add_edge(v2, v3, 100, 10);
  const auto e34     = network.add_edge(v3, v4, 100, 10);
  network.add_successor(e01, e12);
  network.add_successor(e12, e23);
  network.add_successor(e23, e34);
  instance.add_train("Tr1", 10, 10, 2, 2, true, 0, 10, v0, 0, 10, v4, 1);
  instance.add_empty_station("S");
  instance.add_track_to_station("S", e12);
  instance.add_track_to_station("S", e23);
  instance.insert_stop("Tr1", "S", 0, 30);

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {});
  auto&                                  stop = solver.m_vars["stop"];
  std::vector<std::pair<GRBVar, double>> values;
  for (const auto e : {e01, e12, e23, e34}) {
    values.emplace_back(solver.m_vars["x"](0, e), 1);
  }
  auto values_v3 = values;
  values_v3.emplace_back(stop(0, 0, v2), 0.2);
  values_v3.emplace_back(stop(0, 0, v3), 0.8);
  auto values_v2 = values;
  values_v2.emplace_back(stop(0, 0, v2), 0.8);
  values_v2.emplace_back(stop(0, 0, v3), 0.2);

  EXPECT_EQ(solver.relaxation_state(relaxation(values_v3)).stop_positions.at(0),
            std::vector<double>{300});
  EXPECT_EQ(solver.relaxation_state(relaxation(values_v2)).stop_positions.at(0),
            std::vector<double>{200});
}

TEST(GenPOMovingBlockMIPSolver, RelaxationStateCutsRouteBackToPreviousStop) {
  // Tr1 stops at S along the short route and then at S2 along the long one.
  // On the short route, it cannot serve S2, hence its route is cut back to S,
  // and the completion routes the rest.
  auto routes = two_routes();
  routes.instance.add_empty_station("S2");
  routes.instance.add_track_to_station("S2", routes.long_route.at(2));
  routes.instance.insert_stop("Tr1", "S", 0, 30);
  routes.instance.insert_stop("Tr1", "S2", 60, 90);
  const auto& s = routes.short_route;

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {});
  std::vector<std::pair<GRBVar, double>> values;
  for (const auto e : s) {
    values.emplace_back(solver.m_vars["x"](0, e), 1);
  }
  const auto state = solver.relaxation_state(relaxation(values));

  EXPECT_EQ(state.train_edges.at(0),
            (cda_rail::index_vector{s.at(0), s.at(1)}));
  EXPECT_EQ(state.stop_positions.at(0), std::vector<double>{200});
}

TEST(GenPOMovingBlockMIPSolver, CompleteRelaxationKeepsOnlyBetterSolutions) {
  // The long routes give the first incumbent, which the same routes do not
  // replace, but the short ones do, since they are quicker
  const auto routes = two_routes();

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {}, {});
  const auto values_of = [&solver](const cda_rail::index_vector& route) {
    std::vector<std::pair<GRBVar, double>> values;
    for (const auto e : route) {
      values.emplace_back(solver.m_vars["x"](0, e), 1);
      values.emplace_back(solver.m_vars["x"](1, e), 1);
    }
    return relaxation(values);
  };

  EXPECT_EQ(solver.complete_relaxation(values_of(routes.long_route)), true);
  ASSERT_TRUE(solver.m_simulated_incumbent.has_value());
  const auto long_obj = solver.m_simulated_incumbent->obj;
  EXPECT_EQ(solver.m_simulated_incumbent->state.train_edges.at(0),
            routes.long_route);
  EXPECT_EQ(solver.complete_relaxation(values_of(routes.long_route)), false);
  EXPECT_EQ(solver.complete_relaxation(values_of(routes.short_route)), true);
  EXPECT_LT(solver.m_simulated_incumbent->obj, long_obj);
  EXPECT_EQ(solver.m_simulated_incumbent->state.train_edges.at(0),
            routes.short_route);

  // No train can exit on time, which a maximal exit delay of 0 demands
  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver bounded(
      routes.instance);
  bounded.solve_init_general_mip(false, true);
  bounded.create_model({}, {}, {.max_exit_delay = 0});
  EXPECT_EQ(bounded.complete_relaxation(relaxation({})), std::nullopt);
  EXPECT_FALSE(bounded.m_simulated_incumbent.has_value());

  // Two trains enter the single track between v0 and v1 from opposite ends
  // at the same time, hence the completion fails
  cda_rail::instances::GeneralPerformanceOptimizationInstance simultaneous;
  auto&      network = simultaneous.get_editable_network();
  const auto v0      = network.add_vertex("v0", cda_rail::VertexType::TTD);
  const auto v1      = network.add_vertex("v1", cda_rail::VertexType::TTD);
  network.add_edge(v0, v1, 100, 10);
  network.add_edge(v1, v0, 100, 10);
  simultaneous.add_train("Tr1", 10, 10, 2, 2, true, 0, 10, v0, 0, 10, v1, 1);
  simultaneous.add_train("Tr2", 10, 10, 2, 2, true, 0, 10, v1, 0, 10, v0, 1);
  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver simultaneous_solver(
      simultaneous);
  simultaneous_solver.solve_init_general_mip(false, true);
  simultaneous_solver.create_model({}, {}, {});
  EXPECT_EQ(simultaneous_solver.complete_relaxation(relaxation({})),
            std::nullopt);
}

TEST(GenPOMovingBlockMIPSolver, SimulatedIncumbentOnlyReplacesWorseSolutions) {
  // Gurobi's optimal solution is kept if it is strictly better than the
  // simulated incumbent, whose objective is set here, and replaced otherwise.
  // Gurobi's bound is then reduced to the incumbent.
  const auto routes = two_routes();

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {.use_lazy_constraints = false, .abs_mip_gap = 1e-4},
                      {});
  auto simulated =
      solver.simulate_state(solver.completed_initial_state().value());
  ASSERT_TRUE(simulated.has_value());
  solver.m_model->optimize();
  cda_rail::instances::SolGeneralPerformanceOptimizationInstance grb(
      routes.instance);
  solver.extract_solution(grb);
  ASSERT_EQ(grb.get_status(), cda_rail::SolutionStatus::Optimal);
  ASSERT_GT(grb.get_obj(), 0);

  const auto with_incumbent = [&](double obj) {
    simulated->obj               = obj;
    solver.m_simulated_incumbent = simulated;
    cda_rail::instances::SolGeneralPerformanceOptimizationInstance sol(
        routes.instance);
    solver.extract_solution(sol);
    const bool replaced = solver.use_simulated_incumbent(sol);
    return std::pair{replaced, sol};
  };

  const auto [worse_replaced, worse] = with_incumbent(grb.get_obj() + 1);
  EXPECT_FALSE(worse_replaced);
  EXPECT_EQ(worse.get_obj(), grb.get_obj());
  EXPECT_EQ(worse.get_lower_bound(), grb.get_lower_bound());
  EXPECT_EQ(worse.get_status(), cda_rail::SolutionStatus::Optimal);

  const auto [tie_replaced, tie] = with_incumbent(grb.get_obj());
  EXPECT_TRUE(tie_replaced);
  EXPECT_EQ(tie.get_obj(), grb.get_obj());
  EXPECT_EQ(tie.get_const_solution_routes().get_route("Tr1").get_edges(),
            simulated->state.train_edges.at(0));
  EXPECT_EQ(tie.get_status(), cda_rail::SolutionStatus::Optimal);

  const auto [better_replaced, better] = with_incumbent(grb.get_obj() / 2);
  EXPECT_TRUE(better_replaced);
  EXPECT_EQ(better.get_obj(), grb.get_obj() / 2);
  EXPECT_EQ(better.get_lower_bound(), grb.get_obj() / 2);
  EXPECT_EQ(better.get_status(), cda_rail::SolutionStatus::Optimal);
  solver.cleanup();
}

TEST(GenPOMovingBlockMIPSolver, SimulatedIncumbentWithoutGurobiBound) {
  // Gurobi stops before it has a non-negative bound, hence the bound is the
  // trivial one of the non-negative objective, and the simulated solution is
  // only feasible
  const auto routes = two_routes();

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  solver.solve_init_general_mip(false, true);
  solver.create_model({}, {.use_lazy_constraints = false}, {});
  solver.m_simulated_incumbent =
      solver.simulate_state(solver.completed_initial_state().value());
  ASSERT_TRUE(solver.m_simulated_incumbent.has_value());
  solver.m_model->set(GRB_DoubleParam_TimeLimit, 0);
  solver.m_model->optimize();
  EXPECT_LT(solver.m_model->get(GRB_DoubleAttr_ObjBound), 0);

  cda_rail::instances::SolGeneralPerformanceOptimizationInstance sol(
      routes.instance);
  solver.extract_solution(sol);
  EXPECT_TRUE(solver.use_simulated_incumbent(sol));
  EXPECT_EQ(sol.get_lower_bound(), 0);
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Feasible);
  solver.cleanup();
}

TEST(GenPOMovingBlockMIPSolver, CompletionStopsOnceIncumbentWithinGap) {
  // With an absolute gap of 1e6, the simulated incumbent is within the gap of
  // any bound. Hence, the callback interrupts Gurobi before it finds a
  // solution, and the simulated one is optimal.
  const auto routes = two_routes();

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(
      routes.instance);
  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback callback(
      &solver);
  solver.solve_init_general_mip(false, true, &callback);
  solver.create_model({}, {.abs_mip_gap = 1e6, .completion_interval = 1}, {});
  solver.m_simulated_incumbent =
      solver.simulate_state(solver.completed_initial_state().value());
  ASSERT_TRUE(solver.m_simulated_incumbent.has_value());
  solver.m_model->set(GRB_IntParam_LazyConstraints, 1);
  solver.m_model->optimize();
  EXPECT_EQ(solver.m_model->get(GRB_IntAttr_Status), GRB_INTERRUPTED);

  cda_rail::instances::SolGeneralPerformanceOptimizationInstance sol(
      routes.instance);
  solver.extract_solution(sol);
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Timeout);
  EXPECT_TRUE(solver.use_simulated_incumbent(sol));
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  solver.cleanup();
}

// NOLINTEND (clang-analyzer-deadcode.DeadStores)
