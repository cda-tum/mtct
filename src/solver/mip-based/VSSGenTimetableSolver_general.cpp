#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "MultiArray.hpp"
#include "VSSModel.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "plog/Init.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Log.h>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using std::size_t;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-unchecked-optional-access)

// NOLINTBEGIN(performance-inefficient-string-concatenation)

cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance
cda_rail::solver::mip_based::VSSGenTimetableSolver::solve(
    const ModelDetail& model_detail, const ModelSettings& model_settings,
    const SolverStrategy&   solver_strategy,
    const SolutionSettings& solution_settings, int time_limit, bool debug_input,
    bool overwrite_severity) {
  /**
   * Solves initiated GeneralPerformanceOptimizationInstance m_instance using
   * Gurobi and a flexible MILP formulation. The level of detail can be
   * controlled using the parameters.
   *
   * @param model_detail: Contains information on the m_model detail, namely
   * - delta_t: Length of discretized time intervals in seconds. Default: 15
   * - fix_routes: If true, the routes are fixed to the ones given in the
   * - train_dynamic: If true, the train dynamics (i.e., limited acceleration
   * and deceleration) are included in the m_model. Default: true
   * - braking_curves: If true, the braking curves (i.e., the braking distance
   * depending on the current speed has to be cleared) are included in the
   * m_model. Default: true
   *
   * @param model_settings: Contains information on the m_model settings, namely
   * - model_type: Denotes, how the VSS borders are modelled in the solution
   * process. Default uses VSSModel::Continuous
   * - use_pwl: If true, the braking distances are approximated by piecewise
   * linear functions with a fixed maximal error. Otherwise, they are modeled as
   * quadratic functions and Gurobi's ability to solve these using spatial
   * branching is used. Only relevant if include_braking_curves_input is true.
   * Default: false
   * - use_schedule_cuts: If true, the formulation is strengthened using cuts
   * implied by the schedule. Default: true
   *
   * @param solver_strategy: Specify information on the algorithm's strategy to
   * use, namely
   * - iterative_approach: If true, the VSS is iterated to optimality. Default:
   * false
   * - optimality_strategy: Specify the optimality strategy to use. Default:
   * Optimal
   * - update_strategy: Specify the update strategy to use. Only relevant if
   * iterative approach is used. Default: Fixed
   * - initial_value: Specify the initial value or fraction to use. Only
   * relevant if iterative approach is used. In case of fixed update, the value
   * has to be an integer. Otherwise between 0 and 1. Default: 1
   * - update_value: Specify the update value or fraction to use. Only relevant
   * if iterative approach is used. In case of fixed update, the value has to be
   * greater than 1, otherwise between 0 and 1. Default: 2
   *
   * @param solution_settings: Specify information on the solution, namely
   * - postprocess: If true, the solution is postprocessed to remove potentially
   * unused VSS. Default: false
   * - export_option: Denotes if the solution and/or Gurobi m_model is exported.
   * Default: NoExport
   * - name: Name of the file (without extension) to which the m_model is
   * exported. Default: "m_model"
   * - path: Path to which the m_model is exported. Default: "", i.e., the
   * current working directory
   *
   * @param time_limit: Time limit in seconds. No limit if negative. Default: -1
   *
   * @param debug: If true, (more detailed) debug output is printed. Default:
   * false
   * @param overwrite_severity: If true, the severity of the log is overwritten
   * even if this decreases the logging level. Default: true
   *
   * @return Solution object containing status, objective value, and solution
   */

  auto old_instance = initialize_variables(
      model_detail, model_settings, solver_strategy, solution_settings,
      time_limit, debug_input, overwrite_severity);

  create_variables();
  set_objective();
  create_constraints();

  set_timeout(time_limit);

  const auto sol_object = optimize(old_instance, time_limit);

  export_lp_if_applicable(solution_settings);

  if (old_instance.has_value()) {
    m_instance = old_instance.value();
  }

  export_solution_if_applicable(sol_object, solution_settings);

  cleanup();

  return sol_object.value();
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_variables() {
  /**
   * Creates general variables that are independent of the fixed route
   */

  m_vars["v"] = MultiArray<GRBVar>(num_tr, num_t + 1);
  m_vars["x"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);
  m_vars["x_sec"] =
      MultiArray<GRBVar>(num_tr, num_t, unbreakable_sections.size());
  m_vars["y_sec_fwd"] = MultiArray<GRBVar>(num_t, fwd_bwd_sections.size());
  m_vars["y_sec_bwd"] = MultiArray<GRBVar>(num_t, fwd_bwd_sections.size());

  if (vss_model.get_only_stop_at_vss()) {
    m_vars["stopped"] = MultiArray<GRBVar>(num_tr, num_t);
  }

  auto train_list = m_instance.get_const_train_list();
  for (size_t i = 0; i < num_tr; ++i) {
    auto max_speed =
        m_instance.get_const_train_list().get_train(i).get_max_speed();
    auto tr_name = train_list.get_train(i).get_name();
    for (size_t t = train_interval[i].first; t <= train_interval[i].second + 1;
         ++t) {
      m_vars["v"](i, t) =
          m_model->addVar(0, max_speed, 0, GRB_CONTINUOUS,
                          "v_" + tr_name + "_" + std::to_string(t * dt));
    }
    for (size_t t = train_interval[i].first; t <= train_interval[i].second;
         ++t) {
      for (auto const edge_id :
           m_instance.edges_used_by_train(tr_name, fix_routes)) {
        const auto& edge = m_instance.get_editable_network().get_edge(edge_id);
        const auto& edge_name =
            "[" +
            m_instance.get_editable_network().get_vertex(edge.source).name +
            "," +
            m_instance.get_editable_network().get_vertex(edge.target).name +
            "]";
        m_vars["x"](i, t, edge_id) = m_model->addVar(
            0, 1, 0, GRB_BINARY,
            "x_" + tr_name + "_" + std::to_string(t * dt) + "_" + edge_name);
      }
      for (const auto& sec : unbreakable_section_indices(i)) {
        m_vars["x_sec"](i, t, sec) =
            m_model->addVar(0, 1, 0, GRB_BINARY,
                            "x_sec_" + tr_name + "_" + std::to_string(t * dt) +
                                "_" + std::to_string(sec));
      }
    }
  }
  for (size_t t = 0; t < num_t; ++t) {
    for (size_t i = 0; i < fwd_bwd_sections.size(); ++i) {
      m_vars["y_sec_fwd"](t, i) = m_model->addVar(
          0, 1, 0, GRB_BINARY,
          "y_sec_fwd_" + std::to_string(t * dt) + "_" + std::to_string(i));
      m_vars["y_sec_bwd"](t, i) = m_model->addVar(
          0, 1, 0, GRB_BINARY,
          "y_sec_bwd_" + std::to_string(t * dt) + "_" + std::to_string(i));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_discretized_variables() {
  /**
   * Creates variables connected to the VSS decisions of the problem
   */

  m_vars["b"] = MultiArray<GRBVar>(no_border_vss_vertices.size());

  for (size_t i = 0; i < no_border_vss_vertices.size(); ++i) {
    const auto& v_name = m_instance.get_editable_network()
                             .get_vertex(no_border_vss_vertices[i])
                             .name;
    m_vars["b"](i)     = m_model->addVar(0, 1, 0, GRB_BINARY, "b_" + v_name);
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_variables() {
  /**
   * This method creates the variables needed if the graph is not discretized.
   */

  size_t max_vss = 0;
  for (const auto& e : breakable_edges) {
    max_vss = std::max(
        max_vss, static_cast<size_t>(
                     m_instance.get_editable_network().max_vss_on_edge(e)));
  }

  m_vars["b_pos"] = MultiArray<GRBVar>(num_breakable_sections, max_vss);
  m_vars["b_front"] =
      MultiArray<GRBVar>(num_tr, num_t, num_breakable_sections, max_vss);
  m_vars["b_rear"] =
      MultiArray<GRBVar>(num_tr, num_t, num_breakable_sections, max_vss);

  if (this->vss_model.get_model_type() == vss::ModelType::Inferred) {
    m_vars["num_vss_segments"]  = MultiArray<GRBVar>(relevant_edges.size());
    m_vars["frac_vss_segments"] = MultiArray<GRBVar>(
        relevant_edges.size(),
        this->vss_model.get_separation_functions().size(), max_vss);
    m_vars["edge_type"] =
        MultiArray<GRBVar>(relevant_edges.size(),
                           this->vss_model.get_separation_functions().size());
    m_vars["frac_type"] = MultiArray<GRBVar>(
        relevant_edges.size(),
        this->vss_model.get_separation_functions().size(), max_vss);
  } else if (this->vss_model.get_model_type() == vss::ModelType::Continuous) {
    m_vars["b_used"] = MultiArray<GRBVar>(relevant_edges.size(), max_vss);
  } else if (this->vss_model.get_model_type() == vss::ModelType::InferredAlt) {
    m_vars["type_num_vss_segments"] = MultiArray<GRBVar>(
        relevant_edges.size(),
        this->vss_model.get_separation_functions().size(), max_vss);
  } else {
    throw exceptions::ConsistencyException(
        "Model type not supported for non-discretized graph");
  }

  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e = breakable_edges[i];
    const auto  vss_number_e =
        m_instance.get_editable_network().max_vss_on_edge(e);
    const auto& edge     = m_instance.get_editable_network().get_edge(e);
    const auto  edge_len = edge.length;
    const auto& edge_name =
        "[" + m_instance.get_editable_network().get_vertex(edge.source).name +
        "," + m_instance.get_editable_network().get_vertex(edge.target).name +
        "]";
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      const auto& lb = 0;
      const auto& ub = edge_len;
      m_vars["b_pos"](i, vss) =
          m_model->addVar(lb, ub, 0, GRB_CONTINUOUS,
                          "b_pos_" + edge_name + "_" + std::to_string(vss));
      for (const size_t tr :
           m_instance.all_trains_on_edge(e, this->fix_routes, false)) {
        for (size_t t = train_interval[tr].first;
             t <= train_interval[tr].second; ++t) {
          m_vars["b_front"](tr, t, i, vss) = m_model->addVar(
              0, 1, 0, GRB_BINARY,
              "b_front_" + std::to_string(tr) + "_" + std::to_string(t * dt) +
                  "_" + edge_name + "_" + std::to_string(vss));
          if (m_instance.get_const_train_list().get_train(tr).has_tim()) {
            m_vars["b_rear"](tr, t, i, vss) = m_model->addVar(
                0, 1, 0, GRB_BINARY,
                "b_rear_" + std::to_string(tr) + "_" + std::to_string(t * dt) +
                    "_" + edge_name + "_" + std::to_string(vss));
          }
        }
      }
    }
  }

  for (size_t i = 0; i < relevant_edges.size(); ++i) {
    const auto& e = relevant_edges[i];
    const auto  vss_number_e =
        m_instance.get_editable_network().max_vss_on_edge(e);
    const auto& edge = m_instance.get_editable_network().get_edge(e);
    const auto& edge_name =
        "[" + m_instance.get_editable_network().get_vertex(edge.source).name +
        "," + m_instance.get_editable_network().get_vertex(edge.target).name +
        "]";

    if (this->vss_model.get_model_type() == vss::ModelType::Inferred) {
      m_vars["num_vss_segments"](i) = m_model->addVar(
          1, vss_number_e + 1, 0, GRB_INTEGER, "num_vss_segments_" + edge_name);

      if (iterative_vss &&
          vss_number_e + 1 > max_vss_per_edge_in_iteration.at(i)) {
        m_vars["num_vss_segments"](i).set(
            GRB_DoubleAttr_UB,
            static_cast<double>(max_vss_per_edge_in_iteration.at(i)) + 1);
      }

      for (size_t sep_type = 0;
           sep_type < this->vss_model.get_separation_functions().size();
           ++sep_type) {
        m_vars["edge_type"](i, sep_type) = m_model->addVar(
            0, 1, 0, GRB_BINARY,
            "edge_type_" + edge_name + "_" + std::to_string(sep_type));
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          const auto& lb                                = 0.0;
          const auto& ub                                = 1.0;
          m_vars["frac_vss_segments"](i, sep_type, vss) = m_model->addVar(
              lb, ub, 0, GRB_CONTINUOUS,
              "frac_vss_segments_" + edge_name + "_" +
                  std::to_string(sep_type) + "_" + std::to_string(vss));
          m_vars["frac_type"](i, sep_type, vss) = m_model->addVar(
              lb, ub, 0, GRB_CONTINUOUS,
              "frac_type_" + edge_name + "_" + std::to_string(sep_type) + "_" +
                  std::to_string(vss));
        }
      }
    } else if (this->vss_model.get_model_type() == vss::ModelType::Continuous) {
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        m_vars["b_used"](i, vss) =
            m_model->addVar(0, 1, 0, GRB_BINARY,
                            "b_used_" + edge_name + "_" + std::to_string(vss));
        if (iterative_vss && vss >= max_vss_per_edge_in_iteration.at(i)) {
          m_vars["b_used"](i, vss).set(GRB_DoubleAttr_UB, 0);
        }
      }
    } else if (this->vss_model.get_model_type() ==
               vss::ModelType::InferredAlt) {
      for (size_t sep_type = 0;
           sep_type < this->vss_model.get_separation_functions().size();
           ++sep_type) {
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          m_vars["type_num_vss_segments"](i, sep_type, vss) = m_model->addVar(
              0, 1, 0, GRB_BINARY,
              "type_num_vss_segments_" + edge_name + "_" +
                  std::to_string(sep_type) + "_" + std::to_string(vss));

          if (iterative_vss && vss >= max_vss_per_edge_in_iteration.at(i)) {
            m_vars["type_num_vss_segments"](i, sep_type, vss)
                .set(GRB_DoubleAttr_UB, 0);
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_only_stop_at_vss_variables() {
  size_t max_vss = 0;
  for (const auto& e : breakable_edges) {
    max_vss = std::max(
        max_vss, static_cast<size_t>(
                     m_instance.get_editable_network().max_vss_on_edge(e)));
  }

  m_vars["b_tight"] =
      MultiArray<GRBVar>(num_tr, num_t, num_breakable_sections, max_vss);
  m_vars["e_tight"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);

  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e = breakable_edges[i];
    const auto  vss_number_e =
        m_instance.get_editable_network().max_vss_on_edge(e);
    const auto& edge = m_instance.get_editable_network().get_edge(e);
    const auto& edge_name =
        "[" + m_instance.get_editable_network().get_vertex(edge.source).name +
        "," + m_instance.get_editable_network().get_vertex(edge.target).name +
        "]";
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      for (const size_t tr :
           m_instance.all_trains_on_edge(e, this->fix_routes, false)) {
        const auto& tr_name =
            m_instance.get_const_train_list().get_train(tr).get_name();
        for (size_t t = train_interval[tr].first + 2;
             t <= train_interval[tr].second; ++t) {
          m_vars["b_tight"](tr, t, i, vss) = m_model->addVar(
              0, 1, 0, GRB_BINARY,
              "b_tight_" + tr_name + "_" + std::to_string(t * dt) + "_" +
                  edge_name + "_" + std::to_string(vss));
        }
      }
    }
  }

  for (size_t e = 0; e < num_edges; ++e) {
    const auto& edge = m_instance.get_editable_network().get_edge(e);
    const auto& edge_name =
        "[" + m_instance.get_editable_network().get_vertex(edge.source).name +
        "," + m_instance.get_editable_network().get_vertex(edge.target).name +
        "]";
    for (const size_t tr :
         m_instance.all_trains_on_edge(e, this->fix_routes, false)) {
      const auto& tr_name =
          m_instance.get_const_train_list().get_train(tr).get_name();
      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        m_vars["e_tight"](tr, t, e) =
            m_model->addVar(0, 1, 0, GRB_BINARY,
                            "e_tight_" + tr_name + "_" +
                                std::to_string(t * dt) + "_" + edge_name);
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::set_objective() {
  /**
   * Sets the objective function of the problem
   */

  PLOGD << "Set objective";

  // sum over all b_i as in no_border_vss_vertices
  m_objective_expr = 0;
  if (vss_model.get_model_type() == vss::ModelType::Discrete) {
    for (size_t i = 0; i < no_border_vss_vertices.size(); ++i) {
      m_objective_expr += m_vars["b"](i);
    }
  } else if (vss_model.get_model_type() == vss::ModelType::Continuous) {
    for (size_t i = 0; i < relevant_edges.size(); ++i) {
      const auto& e = relevant_edges[i];
      const auto  vss_number_e =
          m_instance.get_editable_network().max_vss_on_edge(e);
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        m_objective_expr += m_vars["b_used"](i, vss);
      }
    }
  } else if (vss_model.get_model_type() == vss::ModelType::Inferred) {
    for (size_t i = 0; i < relevant_edges.size(); ++i) {
      m_objective_expr += (m_vars["num_vss_segments"](i) - 1);
    }
  } else if (vss_model.get_model_type() == vss::ModelType::InferredAlt) {
    for (size_t i = 0; i < relevant_edges.size(); ++i) {
      const auto& e = relevant_edges[i];
      const auto  vss_number_e =
          m_instance.get_editable_network().max_vss_on_edge(e);
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        for (size_t sep_type = 0;
             sep_type < this->vss_model.get_separation_functions().size();
             ++sep_type) {
          m_objective_expr += (static_cast<double>(vss) + 1) *
                              m_vars["type_num_vss_segments"](i, sep_type, vss);
        }
      }
    }
  } else {
    throw std::logic_error("Objective for vss m_model type not implemented");
  }
  m_model->setObjective(m_objective_expr, GRB_MINIMIZE);
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_discretized_constraints() {
  /**
   * Creates VSS constraints, i.e., on NoBorderVSS sections two trains must be
   * separated by a chosen vertex.
   */

  for (const auto& no_border_vss_section : no_border_vss_sections) {
    const auto section_set = cda_rail::index_set(no_border_vss_section.begin(),
                                                 no_border_vss_section.end());
    const auto tr_on_section = m_instance.trains_in_section(section_set);
    const auto no_border_vss_section_sorted =
        m_instance.get_editable_network().combine_reverse_edges(
            no_border_vss_section, true);
    auto tr_on_section_vec =
        cda_rail::index_vector(tr_on_section.begin(), tr_on_section.end());
    std::ranges::sort(tr_on_section_vec);
    for (size_t i = 0; i < tr_on_section_vec.size(); ++i) {
      const auto tr1          = tr_on_section_vec[i];
      const auto tr1_interval = train_interval[tr1];
      const auto tr1_name =
          m_instance.get_const_train_list().get_train(tr1).get_name();
      const auto tr1_route = m_instance.get_const_routes().get_route(tr1_name);
      for (size_t j = i + 1; j < tr_on_section_vec.size(); ++j) {
        const auto tr2          = tr_on_section_vec[j];
        const auto tr2_interval = train_interval[tr2];
        const auto tr2_name =
            m_instance.get_const_train_list().get_train(tr2).get_name();
        const auto tr2_route =
            m_instance.get_const_routes().get_route(tr2_name);
        std::pair<size_t, size_t> t_interval;
        t_interval.first  = std::max(tr1_interval.first, tr2_interval.first);
        t_interval.second = std::min(tr1_interval.second, tr2_interval.second);
        for (size_t t = t_interval.first; t <= t_interval.second; ++t) {
          for (size_t e1 = 0; e1 < no_border_vss_section_sorted.size(); ++e1) {
            for (size_t e2 = 0; e2 < no_border_vss_section_sorted.size();
                 ++e2) {
              if (e1 == e2) {
                continue;
              }
              GRBLinExpr lhs        = 2;
              GRBLinExpr lhs_first  = 0;
              GRBLinExpr lhs_second = 0;
              const auto e1_first   = no_border_vss_section_sorted[e1].first;
              const auto e1_second  = no_border_vss_section_sorted[e1].second;
              const auto e2_first   = no_border_vss_section_sorted[e2].first;
              const auto e2_second  = no_border_vss_section_sorted[e2].second;
              if (tr1_route.contains_edge(e1_first)) {
                lhs -= m_vars["x"](tr1, t, e1_first.value());
                lhs_first += m_vars["x"](tr1, t, e1_first.value());
              }
              if (tr1_route.contains_edge(e1_second)) {
                lhs -= m_vars["x"](tr1, t, e1_second.value());
                lhs_second += m_vars["x"](tr1, t, e1_second.value());
              }
              if (tr2_route.contains_edge(e2_first)) {
                lhs -= m_vars["x"](tr2, t, e2_first.value());
                lhs_first += m_vars["x"](tr2, t, e2_first.value());
              }
              if (tr2_route.contains_edge(e2_second)) {
                lhs -= m_vars["x"](tr2, t, e2_second.value());
                lhs_second += m_vars["x"](tr2, t, e2_second.value());
              }

              for (size_t e_overlap = std::min(e1, e2);
                   e_overlap < std::max(e1, e2); ++e_overlap) {
                const auto left_pair = no_border_vss_section_sorted[e_overlap];
                const auto right_pair =
                    no_border_vss_section_sorted[e_overlap + 1];
                const auto left_edge  = left_pair.first.has_value()
                                            ? left_pair.first.value()
                                            : left_pair.second.value();
                const auto right_edge = right_pair.first.has_value()
                                            ? right_pair.first.value()
                                            : right_pair.second.value();
                const auto v_overlap =
                    m_instance.get_editable_network().common_vertex(left_edge,
                                                                    right_edge);
                if (!v_overlap.has_value()) {
                  throw exceptions::ConsistencyException(
                      "No common vertex found, this should not have happened");
                }

                auto const v_overlap_index =
                    std::ranges::find(no_border_vss_vertices,
                                      v_overlap.value()) -
                    no_border_vss_vertices.begin();
                if (v_overlap_index >= no_border_vss_vertices.size()) {
                  throw exceptions::ConsistencyException(
                      "Vertex not found in no_border_vss_vertices, this should "
                      "not have happened");
                }
                lhs += m_vars["b"](v_overlap_index);
              }

              m_model->addConstr(lhs >= 1,
                                 "vss_" + tr1_name + "_" + tr2_name + "_" +
                                     std::to_string(t) + "_" +
                                     std::to_string(e1_first.value()) + "_" +
                                     std::to_string(e2_first.value()));

              if ((!m_instance.get_const_train_list()
                        .get_train(tr1)
                        .has_tim() &&
                   (e1 > e2)) ||
                  (!m_instance.get_const_train_list()
                        .get_train(tr2)
                        .has_tim() &&
                   (e2 > e1))) {
                m_model->addConstr(lhs_first <= 1,
                                   "vss_tim_first_" + tr1_name + "_" +
                                       tr2_name + "_" + std::to_string(t) +
                                       "_" + std::to_string(e1_first.value()) +
                                       "_" + std::to_string(e2_first.value()) +
                                       "_first");
              }
              if ((!m_instance.get_const_train_list()
                        .get_train(tr2)
                        .has_tim() &&
                   (e1 > e2)) ||
                  (!m_instance.get_const_train_list()
                        .get_train(tr1)
                        .has_tim() &&
                   (e2 > e1))) {
                m_model->addConstr(lhs_second <= 1,
                                   "vss_tim_second_" + tr1_name + "_" +
                                       tr2_name + "_" + std::to_string(t) +
                                       "_" + std::to_string(e1_first.value()) +
                                       "_" + std::to_string(e2_first.value()) +
                                       "_first");
              }
            }
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_unbreakable_sections_constraints() {
  /**
   * Creates constraints for unbreakable sections, i.e., only one train can be
   * on an unbreakable section at a time.
   */

  for (size_t sec_index = 0; sec_index < unbreakable_sections.size();
       ++sec_index) {
    const auto&               sec = unbreakable_sections[sec_index];
    const cda_rail::index_set sec_set(sec.begin(), sec.end());
    const auto&               tr_on_sec = m_instance.trains_in_section(sec_set);
    // tr is on section if it occupies at least one edge of the section
    for (auto const tr : tr_on_sec) {
      const auto& tr_interval = train_interval[tr];
      const auto& tr_name =
          m_instance.get_const_train_list().get_train(tr).get_name();
      const auto& tr_route = m_instance.get_const_routes().get_route(tr_name);
      for (size_t t = tr_interval.first; t <= tr_interval.second; ++t) {
        GRBLinExpr lhs   = 0;
        int        count = 0;
        for (auto const e_index : sec) {
          if (tr_route.contains_edge(e_index)) {
            lhs += m_vars["x"](tr, t, e_index);
            count++;
          }
        }
        m_model->addConstr(lhs >= m_vars["x_sec"](tr, t, sec_index),
                           "unbreakable_section_only_" + tr_name + "_" +
                               std::to_string(t) + "_" +
                               std::to_string(sec_index));
        m_model->addConstr(lhs <= count * m_vars["x_sec"](tr, t, sec_index),
                           "unbreakable_section_if_" + tr_name + "_" +
                               std::to_string(t) + "_" +
                               std::to_string(sec_index));
      }
    }

    for (size_t t = 0; t <= num_t; ++t) {
      const auto tr_to_consider =
          m_instance.trains_at_t(static_cast<int>(t) * dt, tr_on_sec);
      GRBLinExpr lhs = 0;
      for (auto const tr : tr_to_consider) {
        lhs += m_vars["x_sec"](tr, t, sec_index);
      }
      m_model->addConstr(lhs <= 1, "unbreakable_section" +
                                       std::to_string(sec_index) +
                                       "_at_most_one_" + std::to_string(t));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_schedule_constraints() {
  /**
   * Creates constraints for general stations, i.e., if a train is in a station:
   * - all other x variables are 0
   * - the speed is 0
   */

  const auto& train_list = m_instance.get_const_train_list();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    const auto  tr_name     = train_list.get_train(tr).get_name();
    const auto& tr_schedule = m_instance.get_const_schedule(tr_name);
    const auto& tr_edges = m_instance.edges_used_by_train(tr, this->fix_routes);
    for (const auto& tr_stop : tr_schedule.get_stops()) {
      const auto  t0 = static_cast<size_t>(tr_stop.get_service_time() / dt);
      const auto  t1 = static_cast<size_t>(std::ceil(
          static_cast<double>(tr_stop.get_earliest_departure()) / dt));
      const auto& stop_station = tr_stop.get_station();
      const auto  stop_edges   = stop_station.tracks;
      cda_rail::index_set inverse_stop_edges;
      for (const auto e : stop_edges) {
        if (!tr_edges.contains(e)) {
          inverse_stop_edges.insert(e);
        }
      }
      for (size_t t = t0 - 1; t <= t1; ++t) {
        if (t >= t0) {
          m_model->addConstr(m_vars["v"](tr, t) == 0, "station_speed_" +
                                                          tr_name + "_" +
                                                          std::to_string(t));
        }
        if (t >= t0 && t < t1) { // because otherwise the front corresponds to
                                 // t1+dt which is allowed outside
          for (auto const e : inverse_stop_edges) {
            m_model->addConstr(m_vars["x"](tr, t, e) == 0,
                               "station_x_" + tr_name + "_" +
                                   std::to_string(t) + "_" + std::to_string(e));
          }
        }
        // At least on station edge must be occupied, this also holds for the
        // leaving and entering time interval
        GRBLinExpr lhs = 0;
        for (auto const e : stop_edges) {
          // If e in tr_edges
          if (std::ranges::contains(tr_edges, e)) {
            lhs += m_vars["x"](tr, t, e);
          }
        }
        m_model->addConstr(lhs >= 1, "station_occupancy_" + tr_name + "_" +
                                         std::to_string(t));
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_acceleration_constraints() {
  /**
   * This method adds constraints connected to acceleration and deceleration of
   * the trains.
   */

  const auto& train_list = m_instance.get_const_train_list();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    // Iterate over all time steps
    const auto& tr_object = train_list.get_train(tr);
    for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
         ++t) {
      // v(t+1) - v(t) <= acceleration * dt
      m_model->addConstr(m_vars["v"](tr, t + 1) - m_vars["v"](tr, t) <=
                             tr_object.get_acceleration() * dt,
                         "acceleration_" + tr_object.get_name() + "_" +
                             std::to_string(t));
      // v(t) - v(t+1) <= deceleration * dt
      m_model->addConstr(m_vars["v"](tr, t) - m_vars["v"](tr, t + 1) <=
                             tr_object.get_deceleration() * dt,
                         "deceleration_" + tr_object.get_name() + "_" +
                             std::to_string(t));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_brakelen_variables() {
  /**
   * This method creates the variables corresponding to breaking distances.
   */

  m_vars["brakelen"] = MultiArray<GRBVar>(num_tr, num_t);
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto  max_break_len = get_max_brakelen(tr);
    const auto& tr_name =
        m_instance.get_const_train_list().get_train(tr).get_name();
    for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
         ++t) {
      m_vars["brakelen"](tr, t) =
          m_model->addVar(0, max_break_len, 0, GRB_CONTINUOUS,
                          "brakelen_" + tr_name + "_" + std::to_string(t * dt));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_constraints() {
  /**
   * These constraints appear in all variants
   */

  create_general_schedule_constraints();
  create_unbreakable_sections_constraints();
  create_general_speed_constraints();
  create_reverse_occupation_constraints();
  create_general_boundary_constraints();

  if (vss_model.get_only_stop_at_vss()) {
    for (size_t tr = 0; tr < num_tr; ++tr) {
      const auto& tr_speed =
          m_instance.get_const_train_list().get_train(tr).get_max_speed();
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        // v(tr,t) = 0 iff stopped(tr,t) = 0 otherwise v(tr,t) >= V_MIN
        m_model->addConstr(m_vars["v"](tr, t), GRB_GREATER_EQUAL,
                           V_MIN * m_vars["stopped"](tr, t),
                           "v_min_" + std::to_string(tr) + "_" +
                               std::to_string(t * dt));
        m_model->addConstr(m_vars["v"](tr, t), GRB_LESS_EQUAL,
                           tr_speed * m_vars["stopped"](tr, t),
                           "v_max_" + std::to_string(tr) + "_" +
                               std::to_string(t * dt));
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_constraints() {
  /**
   * These constraints appear only when the graph is not discretized
   */

  create_non_discretized_general_constraints();
  create_non_discretized_position_constraints();
  if (this->fix_routes) {
    create_non_discretized_fixed_route_constraints();
  } else {
    create_non_discretized_free_route_constraints();
  }
  if (vss_model.get_model_type() == vss::ModelType::Inferred) {
    create_non_discretized_fraction_constraints();
  } else if (vss_model.get_model_type() == vss::ModelType::InferredAlt) {
    create_non_discretized_alt_fraction_constraints();
  }
  if (vss_model.get_only_stop_at_vss()) {
    create_non_discretized_general_only_stop_at_vss_constraints();
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_general_constraints() {
  /**
   * These constraints appear only when the graph is not discretized, but are
   * general enough to appear in all m_model variants.
   */
  // VSS can only be used if it is non-zero
  if (vss_model.get_model_type() == vss::ModelType::Continuous) {
    for (size_t i = 0; i < relevant_edges.size(); ++i) {
      const auto& e       = relevant_edges[i];
      const auto& e_index = breakable_edge_indices[e];
      const auto  vss_number_e =
          m_instance.get_editable_network().max_vss_on_edge(e);
      const auto  e_len = m_instance.get_editable_network().get_edge(e).length;
      const auto& min_block_len_e =
          m_instance.get_editable_network().get_edge(e).min_block_length;
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        m_model->addConstr(e_len * m_vars["b_used"](i, vss), GRB_GREATER_EQUAL,
                           m_vars["b_pos"](e_index, vss),
                           "b_used_" + std::to_string(e) + "_" +
                               std::to_string(vss));
        m_model->addConstr(m_vars["b_pos"](e_index, vss), GRB_GREATER_EQUAL,
                           m_vars["b_used"](i, vss) * min_block_len_e,
                           "b_used_min_value_if_used_" + std::to_string(e) +
                               "_" + std::to_string(vss));
        // Also remove redundant solutions
        if (vss < vss_number_e - 1) {
          m_model->addConstr(m_vars["b_pos"](e_index, vss), GRB_GREATER_EQUAL,
                             m_vars["b_pos"](e_index, vss + 1) +
                                 m_vars["b_used"](i, vss + 1) * min_block_len_e,
                             "b_used_decreasing_" + std::to_string(e) + "_" +
                                 std::to_string(vss));
        }
      }
    }
  }

  // Connect position of reverse edges
  for (const auto& e_pair : breakable_edges_pairs) {
    if (!e_pair.second.has_value() || !e_pair.first.has_value()) {
      continue;
    }
    const auto vss_number_e =
        m_instance.get_editable_network().max_vss_on_edge(e_pair.first.value());
    if (m_instance.get_editable_network().max_vss_on_edge(
            e_pair.second.value()) != vss_number_e) {
      throw exceptions::ConsistencyException(
          "VSS number of edges " + std::to_string(e_pair.first.value()) +
          " and " + std::to_string(e_pair.second.value()) + " do not match");
    }
    const auto& e_len =
        m_instance.get_editable_network().get_edge(e_pair.first.value()).length;
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      m_model->addConstr(
          m_vars["b_pos"](breakable_edge_indices[e_pair.first.value()], vss) +
              m_vars["b_pos"](breakable_edge_indices[e_pair.second.value()],
                              vss),
          GRB_EQUAL, e_len,
          "b_pos_reverse_" + std::to_string(e_pair.first.value()) + "_" +
              std::to_string(vss) + "_" +
              std::to_string(e_pair.second.value()) + "_" +
              std::to_string(vss));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_position_constraints() {
  /**
   * Creates the position constraints related to non-discretized VSS blocks
   */

  // Border only usable by a train if it is on the edge
  for (size_t e_index = 0; e_index < breakable_edges.size(); ++e_index) {
    const auto& e = breakable_edges[e_index];
    for (const auto& tr :
         m_instance.all_trains_on_edge(e, this->fix_routes, false)) {
      const auto vss_number_e =
          m_instance.get_editable_network().max_vss_on_edge(e);
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          // x(tr,t,e) >= b_front(tr,t,e_index,vss)
          m_model->addConstr(m_vars["x"](tr, t, e), GRB_GREATER_EQUAL,
                             m_vars["b_front"](tr, t, e_index, vss),
                             "x_b_front_" + std::to_string(tr) + "_" +
                                 std::to_string(t) + "_" + std::to_string(e) +
                                 "_" + std::to_string(vss));
          // x(tr,t,e) >= b_rear(tr,t,e_index,vss)
          if (m_instance.get_const_train_list().get_train(tr).has_tim()) {
            m_model->addConstr(m_vars["x"](tr, t, e), GRB_GREATER_EQUAL,
                               m_vars["b_rear"](tr, t, e_index, vss),
                               "x_b_rear_" + std::to_string(tr) + "_" +
                                   std::to_string(t) + "_" + std::to_string(e) +
                                   "_" + std::to_string(vss));
          }
        }
      }
    }
  }

  // Correct number of borders
  for (size_t e_index = 0; e_index < breakable_edges.size(); ++e_index) {
    const auto& e = breakable_edges[e_index];
    const auto  vss_number_e =
        m_instance.get_editable_network().max_vss_on_edge(e);
    const auto& tr_on_e =
        m_instance.all_trains_on_edge(e, this->fix_routes, false);
    for (size_t t = 0; t < num_t; ++t) {
      // sum_(tr,vss) b_front(tr, t, e_index, vss) >= sum_(tr) x(tr, t, e) - 1
      // sum_(tr,vss) b_rear(tr, t, e_index, vss) >= sum_(tr) x(tr, t, e) - 1
      GRBLinExpr lhs_front         = 0;
      GRBLinExpr lhs_rear          = 0;
      GRBLinExpr rhs               = -1;
      bool       create_constraint = false;
      for (const auto& tr :
           m_instance.trains_at_t(static_cast<int>(t) * dt, tr_on_e)) {
        create_constraint = true;
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          lhs_front += m_vars["b_front"](tr, t, e_index, vss);
          if (m_instance.get_const_train_list().get_train(tr).has_tim()) {
            lhs_rear += m_vars["b_rear"](tr, t, e_index, vss);
          }
        }
        rhs += m_vars["x"](tr, t, e);
      }
      if (create_constraint) {
        m_model->addConstr(lhs_front, GRB_GREATER_EQUAL, rhs,
                           "b_front_correct_number_" + std::to_string(t) + "_" +
                               std::to_string(e) + "_" +
                               std::to_string(e_index));
        m_model->addConstr(lhs_rear, GRB_GREATER_EQUAL, rhs,
                           "b_rear_correct_number_" + std::to_string(t) + "_" +
                               std::to_string(e) + "_" +
                               std::to_string(e_index));
        // lhs_front = lhs_rear
        m_model->addConstr(lhs_front, GRB_EQUAL, lhs_rear,
                           "b_front_rear_correct_number_equal_" +
                               std::to_string(t) + "_" + std::to_string(e) +
                               "_" + std::to_string(e_index));
      }
    }
  }

  // At most one border used per train
  for (size_t tr = 0; tr < num_tr; ++tr) {
    for (size_t t = train_interval[tr].first; t < train_interval[tr].second;
         ++t) {
      // sum_(e,vss) b_front(tr, t, e_index, vss) <= 1
      // sum_(e,vss) b_rear(tr, t, e_index, vss) <= 1
      GRBLinExpr lhs_front = 0;
      GRBLinExpr lhs_rear  = 0;
      for (const auto& e :
           m_instance.edges_used_by_train(tr, this->fix_routes)) {
        const auto& e_index = breakable_edge_indices[e];
        const auto  vss_number_e =
            m_instance.get_editable_network().max_vss_on_edge(e);
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          lhs_front += m_vars["b_front"](tr, t, e_index, vss);
          if (m_instance.get_const_train_list().get_train(tr).has_tim()) {
            lhs_rear += m_vars["b_rear"](tr, t, e_index, vss);
          }
        }
      }
      m_model->addConstr(lhs_front, GRB_LESS_EQUAL, 1,
                         "b_front_at_most_one_" + std::to_string(tr) + "_" +
                             std::to_string(t));
      m_model->addConstr(lhs_rear, GRB_LESS_EQUAL, 1,
                         "b_rear_at_most_one_" + std::to_string(tr) + "_" +
                             std::to_string(t));
    }
  }

  // A border must be both front and rear or nothing
  for (size_t e_index = 0; e_index < breakable_edges.size(); ++e_index) {
    const auto& e = breakable_edges[e_index];
    const auto  tr_on_e =
        m_instance.all_trains_on_edge(e, this->fix_routes, false);
    const auto vss_number_e =
        m_instance.get_editable_network().max_vss_on_edge(e);
    for (size_t t = 0; t < num_t; ++t) {
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        // sum_tr b_front(tr, t, e_index, vss) = sum_tr b_rear(tr, t, e_index,
        // vss) <= 1
        GRBLinExpr lhs = 0;
        GRBLinExpr rhs = 0;
        for (const auto& tr :
             m_instance.trains_at_t(static_cast<int>(t) * dt, tr_on_e)) {
          lhs += m_vars["b_front"](tr, t, e_index, vss);
          if (m_instance.get_const_train_list().get_train(tr).has_tim()) {
            rhs += m_vars["b_rear"](tr, t, e_index, vss);
          }
        }
        m_model->addConstr(lhs, GRB_EQUAL, rhs,
                           "b_front_rear_" + std::to_string(t) + "_" +
                               std::to_string(e) + "_" + std::to_string(vss));
        m_model->addConstr(rhs, GRB_LESS_EQUAL, 1,
                           "b_front_rear_limit_" + std::to_string(t) + "_" +
                               std::to_string(e) + "_" + std::to_string(vss));
      }
    }
  }

  // A border is only usable if the VSS is used
  for (size_t e_index = 0; e_index < breakable_edges.size(); ++e_index) {
    const auto& e = breakable_edges[e_index];
    for (const auto& tr :
         m_instance.all_trains_on_edge(e, this->fix_routes, false)) {
      const auto vss_number_e =
          m_instance.get_editable_network().max_vss_on_edge(e);
      // Get index of e in relevant_edges array
      const auto find_index       = std::ranges::find(relevant_edges, e);
      auto       e_index_relevant = find_index - relevant_edges.begin();
      // If edge not found check reverse edge
      if (find_index == relevant_edges.end()) {
        const auto reverse_e =
            m_instance.get_editable_network().get_reverse_edge_index(e).value();
        const auto find_index_reverse =
            std::ranges::find(relevant_edges, reverse_e);
        if (find_index_reverse == relevant_edges.end()) {
          throw exceptions::ConsistencyException(
              "Edge " + std::to_string(e) + " and its reverse edge " +
              std::to_string(reverse_e) + " not found in relevant_edges");
        }
        e_index_relevant = find_index_reverse - relevant_edges.begin();
      }

      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          if (vss_model.get_model_type() == vss::ModelType::Continuous) {
            // b_front(tr, t, e_index, vss) <= b_used(e_index_relevant, vss)
            m_model->addConstr(m_vars["b_front"](tr, t, e_index, vss),
                               GRB_LESS_EQUAL,
                               m_vars["b_used"](e_index_relevant, vss),
                               "b_front_b_used_" + std::to_string(tr) + "_" +
                                   std::to_string(t) + "_" + std::to_string(e) +
                                   "_" + std::to_string(vss));
            // b_rear(tr, t, e_index, vss) <= b_used(e_index_relevant, vss)
            if (m_instance.get_const_train_list().get_train(tr).has_tim()) {
              m_model->addConstr(
                  m_vars["b_rear"](tr, t, e_index, vss), GRB_LESS_EQUAL,
                  m_vars["b_used"](e_index_relevant, vss),
                  "b_rear_b_used_" + std::to_string(tr) + "_" +
                      std::to_string(t) + "_" + std::to_string(e) + "_" +
                      std::to_string(vss));
            }
          } else if (vss_model.get_model_type() == vss::ModelType::Inferred) {
            // b_front(tr, t, e_index, vss) <=
            // (num_vss_segments(e_index_relevant) - 1) / (vss + 1)
            m_model->addConstr(
                m_vars["b_front"](tr, t, e_index, vss), GRB_LESS_EQUAL,
                (m_vars["num_vss_segments"](e_index_relevant) - 1) /
                    (static_cast<double>(vss) + 1),
                "b_front_num_vss_segments_" + std::to_string(tr) + "_" +
                    std::to_string(t) + "_" + std::to_string(e) + "_" +
                    std::to_string(vss));
            // b_rear(tr, t, e_index, vss) <=
            // (num_vss_segments(e_index_relevant) - 1) / (vss + 1)
            if (m_instance.get_const_train_list().get_train(tr).has_tim()) {
              m_model->addConstr(
                  m_vars["b_rear"](tr, t, e_index, vss), GRB_LESS_EQUAL,
                  (m_vars["num_vss_segments"](e_index_relevant) - 1) /
                      (static_cast<double>(vss) + 1),
                  "b_rear_num_vss_segments_" + std::to_string(tr) + "_" +
                      std::to_string(t) + "_" + std::to_string(e) + "_" +
                      std::to_string(vss));
            }
          } else if (vss_model.get_model_type() ==
                     vss::ModelType::InferredAlt) {
            // b_front(tr, t, e_index, vss) <= sum
            // type_num_vss_segments(e_index_relevant, *, <= vss)
            GRBLinExpr rhs = 0;
            for (size_t sep_type_index = 0;
                 sep_type_index < vss_model.get_separation_functions().size();
                 ++sep_type_index) {
              for (size_t vss2 = 0; vss2 <= vss; ++vss2) {
                rhs += m_vars["type_num_vss_segments"](e_index_relevant,
                                                       sep_type_index, vss2);
              }
            }
            m_model->addConstr(
                m_vars["b_front"](tr, t, e_index, vss), GRB_LESS_EQUAL, rhs,
                "b_front_num_vss_segments_" + std::to_string(tr) + "_" +
                    std::to_string(t) + "_" + std::to_string(e) + "_" +
                    std::to_string(vss));
            // b_rear(tr, t, e_index, vss) <= sum
            // type_num_vss_segments(e_index_relevant, *, <= vss)
            if (m_instance.get_const_train_list().get_train(tr).has_tim()) {
              m_model->addConstr(
                  m_vars["b_rear"](tr, t, e_index, vss), GRB_LESS_EQUAL, rhs,
                  "b_rear_num_vss_segments_" + std::to_string(tr) + "_" +
                      std::to_string(t) + "_" + std::to_string(e) + "_" +
                      std::to_string(vss));
            }
          }
        }
      }
    }
  }

  // At most one non-tim train can be on any breakable edge
  for (const auto& e : breakable_edges) {
    const auto tr_on_e =
        m_instance.all_trains_on_edge(e, this->fix_routes, false);
    const auto& edge = m_instance.get_editable_network().get_edge(e);
    const auto& v0 = m_instance.get_editable_network().get_vertex(edge.source);
    const auto& v1 = m_instance.get_editable_network().get_vertex(edge.target);
    const auto  e_name = "[" + v0.name + "," + v1.name + "]";
    for (size_t t = 0; t < num_t; ++t) {
      GRBLinExpr lhs = 0;
      for (const auto& tr :
           m_instance.trains_at_t(static_cast<int>(t) * dt, tr_on_e)) {
        if (!m_instance.get_const_train_list().get_train(tr).has_tim()) {
          lhs += m_vars["x"](tr, t, e);
        }
      }
      m_model->addConstr(lhs, GRB_LESS_EQUAL, 1,
                         "non_tim_train_on_edge_" + e_name + "_" +
                             std::to_string(static_cast<int>(t) * dt));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_fraction_constraints() {
  for (size_t i = 0; i < relevant_edges.size(); ++i) {
    const auto& e = relevant_edges[i];
    const auto  vss_number_e =
        m_instance.get_editable_network().max_vss_on_edge(e);
    const auto& edge = m_instance.get_editable_network().get_edge(e);
    const auto& edge_name =
        "[" + m_instance.get_editable_network().get_vertex(edge.source).name +
        "," + m_instance.get_editable_network().get_vertex(edge.target).name +
        "]";
    const auto& breakable_e_index = breakable_edge_indices.at(e);
    const auto  e_len = m_instance.get_editable_network().get_edge(e).length;

    if (vss_model.get_model_type() == vss::ModelType::Inferred) {
      // sum edge_type(i,*) = 1
      GRBLinExpr lhs_sum_edge_type            = 0;
      bool       add_constraint_sum_edge_type = false;
      for (size_t sep_type_index = 0;
           sep_type_index < vss_model.get_separation_functions().size();
           ++sep_type_index) {
        lhs_sum_edge_type += m_vars["edge_type"](i, sep_type_index);
        add_constraint_sum_edge_type = true;
        const auto& sep_func =
            vss_model.get_separation_functions().at(sep_type_index);
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          // NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
          auto const xpts = std::make_unique<double[]>(vss_number_e + 1);
          auto const ypts = std::make_unique<double[]>(vss_number_e + 1);
          // NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
          for (size_t x = 0; x < vss_number_e + 1; ++x) {
            xpts[x] = static_cast<double>(x) + 1;
            ypts[x] = sep_func(vss, x + 1);
          }
          m_model->addGenConstrPWL(
              m_vars["num_vss_segments"](i),
              m_vars["frac_vss_segments"](i, sep_type_index, vss),
              vss_number_e + 1, xpts.get(), ypts.get(),
              "frac_vss_segments_value_constraint_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
        }
      }
      if (add_constraint_sum_edge_type) {
        m_model->addConstr(lhs_sum_edge_type, GRB_EQUAL, 1,
                           "sum_edge_type_" + edge_name);
      }

      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        // b_pos(breakable_e_index, vss) = e_len * sum_{separation_types}
        // frac_type(i, sep_type_index, vss)
        GRBLinExpr lhs = 0;
        for (size_t sep_type_index = 0;
             sep_type_index < vss_model.get_separation_functions().size();
             ++sep_type_index) {
          lhs += m_vars["frac_type"](i, sep_type_index, vss);

          // Make sure that frac_type(i, sep_type_index, vss) =
          // frac_vss_segments(i, sep_type_index, vss) * edge_type(i,
          // sep_type_index) by standard linearization
          const double lb = 0;
          const double ub = 1;
          // frac_type = 0 if edge_type = 0
          m_model->addConstr(
              lb * m_vars["edge_type"](i, sep_type_index), GRB_LESS_EQUAL,
              m_vars["frac_type"](i, sep_type_index, vss),
              "frac_type_0_lb_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
          m_model->addConstr(
              m_vars["frac_type"](i, sep_type_index, vss), GRB_LESS_EQUAL,
              ub * m_vars["edge_type"](i, sep_type_index),
              "frac_type_0_ub_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
          // frac_type = frac_vss_segments if edge_type = 1
          m_model->addConstr(
              (lb - ub) * (1 - m_vars["edge_type"](i, sep_type_index)),
              GRB_LESS_EQUAL,
              m_vars["frac_type"](i, sep_type_index, vss) -
                  m_vars["frac_vss_segments"](i, sep_type_index, vss),
              "frac_type_prod_lb_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
          m_model->addConstr(
              m_vars["frac_type"](i, sep_type_index, vss) -
                  m_vars["frac_vss_segments"](i, sep_type_index, vss),
              GRB_LESS_EQUAL,
              (ub - lb) * (1 - m_vars["edge_type"](i, sep_type_index)),
              "frac_type_prod_ub_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
        }
        lhs *= e_len;
        m_model->addConstr(
            lhs, GRB_EQUAL, m_vars["b_pos"](breakable_e_index, vss),
            "b_pos_limited_" + edge_name + "_" + std::to_string(vss));
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_alt_fraction_constraints() {
  if (vss_model.get_model_type() != vss::ModelType::InferredAlt) {
    return;
  }

  for (size_t i = 0; i < relevant_edges.size(); ++i) {
    const auto& e = relevant_edges[i];
    const auto  vss_number_e =
        m_instance.get_editable_network().max_vss_on_edge(e);
    const auto& edge = m_instance.get_editable_network().get_edge(e);
    const auto& edge_name =
        "[" + m_instance.get_editable_network().get_vertex(edge.source).name +
        "," + m_instance.get_editable_network().get_vertex(edge.target).name +
        "]";
    const auto& breakable_e_index = breakable_edge_indices.at(e);
    const auto  e_len = m_instance.get_editable_network().get_edge(e).length;

    // Only choose one edge type and number per edge
    GRBLinExpr lhs_sum_edge_type = 0;
    for (size_t sep_type_index = 0;
         sep_type_index < vss_model.get_separation_functions().size();
         ++sep_type_index) {
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        lhs_sum_edge_type +=
            m_vars["type_num_vss_segments"](i, sep_type_index, vss);
      }
    }
    m_model->addConstr(lhs_sum_edge_type, GRB_LESS_EQUAL, 1,
                       "sum_edge_vss_type_" + edge_name);

    // Set b_pos accordingly
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      GRBLinExpr rhs = 0;
      for (size_t sep_type_index = 0;
           sep_type_index < vss_model.get_separation_functions().size();
           ++sep_type_index) {
        const auto& sep_func =
            vss_model.get_separation_functions().at(sep_type_index);
        for (size_t num_vss = 1; num_vss <= vss_number_e; ++num_vss) {
          rhs +=
              m_vars["type_num_vss_segments"](i, sep_type_index, num_vss - 1) *
              e_len * sep_func(vss, num_vss + 1);
        }
      }
      m_model->addConstr(
          m_vars["b_pos"](breakable_e_index, vss), GRB_EQUAL, rhs,
          "b_pos_alt_limited_" + edge_name + "_" + std::to_string(vss));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_brakelen_constraints() {
  /**
   * Creates the constraints related to braking distances.
   */
  // break_len(tr, t) = v(tr, t+1)^2 / (2*tr_deceleration)
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_deceleration =
        m_instance.get_const_train_list().get_train(tr).get_deceleration();
    const auto& tr_max_speed =
        m_instance.get_const_train_list().get_train(tr).get_max_speed();
    if (this->use_pwl) {
      const int n = std::ceil(
          tr_max_speed / (2 * std::sqrt(2 * tr_deceleration * ABS_PWL_ERROR)));
      // NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
      auto const xpts = std::make_unique<double[]>(n + 1);
      auto const ypts = std::make_unique<double[]>(n + 1);
      // NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
      for (size_t i = 0; i <= n; ++i) {
        xpts[i] = static_cast<double>(i) * tr_max_speed / n;
        ypts[i] = xpts[i] * xpts[i] / (2 * tr_deceleration);
      }
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        m_model->addGenConstrPWL(
            m_vars["v"](tr, t + 1), m_vars["brakelen"](tr, t), n + 1,
            xpts.get(), ypts.get(),
            "brakelen_" + std::to_string(tr) + "_" + std::to_string(t));
      }
    } else {
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        m_model->addQConstr(m_vars["brakelen"](tr, t), GRB_EQUAL,
                            (1 / (2 * tr_deceleration)) *
                                m_vars["v"](tr, t + 1) * m_vars["v"](tr, t + 1),
                            "brakelen_" + std::to_string(tr) + "_" +
                                std::to_string(t));
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_speed_constraints() {
  /**
   * Train does not exceed maximum speed on edges
   */

  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_speed =
        m_instance.get_const_train_list().get_train(tr).get_max_speed();
    for (const auto e : m_instance.edges_used_by_train(tr, this->fix_routes)) {
      const auto& max_speed =
          m_instance.get_editable_network().get_edge(e).max_speed;
      if (max_speed < tr_speed) {
        for (size_t t = train_interval[tr].first;
             t <= train_interval[tr].second; ++t) {
          // v(tr,t+1) <= max_speed + (tr_speed - max_speed) * (1 - x(tr,t,e))
          m_model->addConstr(
              m_vars["v"](tr, t + 1), GRB_LESS_EQUAL,
              max_speed + (tr_speed - max_speed) * (1 - m_vars["x"](tr, t, e)),
              "v_max_speed_" + std::to_string(tr) + "_" +
                  std::to_string((t + 1) * dt) + "_" + std::to_string(e));
          // If brakelens are included the speed is reduced before entering an
          // edge, otherwise also include v(tr,t) <= max_speed + (tr_speed -
          // max_speed) * (1 - x(tr,t,e))
          if (!this->include_braking_curves) {
            m_model->addConstr(m_vars["v"](tr, t), GRB_LESS_EQUAL,
                               max_speed + (tr_speed - max_speed) *
                                               (1 - m_vars["x"](tr, t, e)),
                               "v_max_speed2_" + std::to_string(tr) + "_" +
                                   std::to_string(t * dt) + "_" +
                                   std::to_string(e));
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_reverse_occupation_constraints() {
  /**
   * A breakable section can only be occupied in one direction at a time. This
   * prevents trains from blocking each other, since reversing trains is not
   * modelled.
   */

  // Connect y_sec and x
  for (size_t t = 0; t < num_t; ++t) {
    const auto tr_at_t = m_instance.trains_at_t(static_cast<int>(t) * dt);
    for (size_t i = 0; i < fwd_bwd_sections.size(); ++i) {
      // y_sec_fwd(t,i) >= x(tr, t, e) for all e in fwd_bwd_sections[i].first
      // and applicable trains y_sec_fwd(t,i) <= sum x(tr, t, e)
      GRBLinExpr rhs = 0;
      for (const auto& e : fwd_bwd_sections[i].first) {
        const auto tr_on_edge =
            m_instance.trains_on_edge(e, this->fix_routes, tr_at_t);
        for (const auto& tr : tr_on_edge) {
          rhs += m_vars["x"](tr, t, e);
          m_model->addConstr(m_vars["y_sec_fwd"](t, i), GRB_GREATER_EQUAL,
                             m_vars["x"](tr, t, e),
                             "y_sec_fwd_linker_1_" + std::to_string(t) + "_" +
                                 std::to_string(i) + "_" + std::to_string(tr) +
                                 "_" + std::to_string(e));
        }
      }
      m_model->addConstr(m_vars["y_sec_fwd"](t, i), GRB_LESS_EQUAL, rhs,
                         "y_sec_fwd_linker_2_" + std::to_string(t) + "_" +
                             std::to_string(i));

      // y_sec_bwd(t,i) >= x(tr, t, e) for all e in fwd_bwd_sections[i].second
      // and applicable trains y_sec_bwd(t,i) <= sum x(tr, t, e)
      rhs = 0;
      for (const auto& e : fwd_bwd_sections[i].second) {
        const auto tr_on_edge =
            m_instance.trains_on_edge(e, this->fix_routes, tr_at_t);
        for (const auto& tr : tr_on_edge) {
          rhs += m_vars["x"](tr, t, e);
          m_model->addConstr(m_vars["y_sec_bwd"](t, i), GRB_GREATER_EQUAL,
                             m_vars["x"](tr, t, e),
                             "y_sec_bwd_linker_1_" + std::to_string(t) + "_" +
                                 std::to_string(i) + "_" + std::to_string(tr) +
                                 "_" + std::to_string(e));
        }
      }
      m_model->addConstr(m_vars["y_sec_bwd"](t, i), GRB_LESS_EQUAL, rhs,
                         "y_sec_bwd_linker_2_" + std::to_string(t) + "_" +
                             std::to_string(i));
    }
  }

  // Only one direction occupied
  for (size_t t = 0; t < num_t; ++t) {
    for (size_t i = 0; i < fwd_bwd_sections.size(); ++i) {
      // y_sec_fwd(t,i) + y_sec_bwd(t, i) <= 1
      m_model->addConstr(
          m_vars["y_sec_fwd"](t, i) + m_vars["y_sec_bwd"](t, i), GRB_LESS_EQUAL,
          1, "y_sec_fwd_bwd_" + std::to_string(t) + "_" + std::to_string(i));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    calculate_fwd_bwd_sections() {
  /**
   * Calculate the forward and backward sections for each breakable section
   */

  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    calculate_fwd_bwd_sections_discretized();
  } else {
    calculate_fwd_bwd_sections_non_discretized();
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    calculate_fwd_bwd_sections_discretized() {
  /**
   * For every section, cluster forward and backward edges.
   */
  for (const auto& vss_section : no_border_vss_sections) {
    const auto vss_section_sorted =
        m_instance.get_editable_network().combine_reverse_edges(vss_section,
                                                                true);
    bool fwd_found = false;
    bool bwd_found = false;
    for (size_t i = 0;
         i < vss_section_sorted.size() && !fwd_found && !bwd_found; ++i) {
      if (vss_section_sorted[i].first.has_value()) {
        fwd_found = true;
      }
      if (vss_section_sorted[i].second.has_value()) {
        bwd_found = true;
      }
    }
    if (!fwd_found || !bwd_found) {
      continue;
    }
    fwd_bwd_sections.emplace_back();
    for (const auto& e : vss_section_sorted) {
      if (e.first.has_value()) {
        fwd_bwd_sections.back().first.emplace_back(e.first.value());
      }
      if (e.second.has_value()) {
        fwd_bwd_sections.back().second.emplace_back(e.second.value());
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    calculate_fwd_bwd_sections_non_discretized() {
  /**
   * For every section, cluster forward and backward edges.
   */
  for (const auto& edge_pair : breakable_edges_pairs) {
    if (!edge_pair.first.has_value() || !edge_pair.second.has_value()) {
      continue;
    }
    fwd_bwd_sections.emplace_back();
    fwd_bwd_sections.back().first.emplace_back(edge_pair.first.value());
    fwd_bwd_sections.back().second.emplace_back(edge_pair.second.value());
  }
}

double cda_rail::solver::mip_based::VSSGenTimetableSolver::get_max_brakelen(
    const size_t& tr) const {
  /**
   * Returns the maximum braking distance of a train.
   */
  const auto& tr_deceleration =
      m_instance.get_const_train_list().get_train(tr).get_deceleration();
  const auto& tr_max_speed =
      m_instance.get_const_train_list().get_train(tr).get_max_speed();
  return tr_max_speed * tr_max_speed / (2 * tr_deceleration);
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_boundary_constraints() {
  /**
   * General boundary conditions, i.e., speed
   */
  auto train_list = m_instance.get_const_train_list();
  for (size_t i = 0; i < num_tr; ++i) {
    auto tr_name = train_list.get_train(i).get_name();
    auto initial_speed =
        m_instance.get_const_schedule(tr_name).get_initial_velocity();
    auto final_speed =
        m_instance.get_const_schedule(tr_name).get_exit_velocity();
    // initial_speed: v(train_interval[i].first) = initial_speed
    m_model->addConstr(m_vars["v"](i, train_interval[i].first) == initial_speed,
                       "initial_speed_" + tr_name);
    // final_speed: v(train_interval[i].second) = final_speed
    m_model->addConstr(m_vars["v"](i, train_interval[i].second + 1) ==
                           final_speed,
                       "final_speed_" + tr_name);
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_only_stop_at_vss_variables() {
  m_vars["stopped"] = MultiArray<GRBVar>(num_tr, num_t);

  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name =
        m_instance.get_const_train_list().get_train(tr).get_name();
    for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
         ++t) {
      m_vars["stopped"](tr, t) =
          m_model->addVar(0, 1, 0, GRB_BINARY,
                          "stopped_" + tr_name + "_" + std::to_string(t * dt));
    }
  }

  if (vss_model.get_model_type() != vss::ModelType::Discrete) {
    create_non_discretized_only_stop_at_vss_variables();
  } else {
    throw exceptions::ConsistencyException(
        "Only stop at VSS variables are not supported for discretized VSS "
        "models");
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_general_only_stop_at_vss_constraints() {
  // At most one b_tight can be true per train and time
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name =
        m_instance.get_const_train_list().get_train(tr).get_name();
    for (size_t t = train_interval[tr].first + 2;
         t <= train_interval[tr].second; ++t) {
      GRBLinExpr lhs = 0;
      for (const auto& e :
           m_instance.edges_used_by_train(tr, this->fix_routes)) {
        if (!m_instance.get_const_network().get_edge(e).breakable) {
          continue;
        }
        const auto& vss_e = m_instance.get_const_network().max_vss_on_edge(e);
        const auto& e_b_index = breakable_edge_indices.at(e);
        for (size_t vss = 0; vss < vss_e; ++vss) {
          lhs += m_vars["b_tight"](tr, t, e_b_index, vss);
        }
      }
      m_model->addConstr(lhs, GRB_LESS_EQUAL, 1,
                         "b_tight_max_one_" + tr_name + "_" +
                             std::to_string(t * dt));
    }
  }

  // On every breakable edge at most one b_tight or e_tight can be one per train
  // and time
  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e     = breakable_edges[i];
    const auto& vss_e = m_instance.get_const_network().max_vss_on_edge(e);
    const auto& edge  = m_instance.get_const_network().get_edge(e);
    const auto& edge_name =
        "[" + m_instance.get_const_network().get_vertex(edge.source).name +
        "," + m_instance.get_const_network().get_vertex(edge.target).name + "]";
    for (const size_t tr :
         m_instance.all_trains_on_edge(e, this->fix_routes, false)) {
      const auto& tr_name =
          m_instance.get_const_train_list().get_train(tr).get_name();
      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        GRBLinExpr lhs = m_vars["e_tight"](tr, t, e);
        for (size_t vss = 0; vss < vss_e; ++vss) {
          lhs += m_vars["b_tight"](tr, t, i, vss);
        }
        m_model->addConstr(lhs, GRB_LESS_EQUAL, 1,
                           "b_tight_e_tight_max_one_" + tr_name + "_" +
                               std::to_string(t * dt) + "_" + edge_name);
      }
    }
  }

  // On every edge at least one b_tight or e_tight must be one if train is
  // present and speed is 0 per train, time, and edge
  for (size_t e = 0; e < num_edges; ++e) {
    const auto& edge = m_instance.get_const_network().get_edge(e);
    const auto& edge_name =
        "[" + m_instance.get_const_network().get_vertex(edge.source).name +
        "," + m_instance.get_const_network().get_vertex(edge.target).name + "]";
    std::optional<size_t> breakable_e_index;
    std::optional<size_t> vss_e;
    if (edge.breakable) {
      breakable_e_index = breakable_edge_indices.at(e);
      vss_e             = m_instance.get_const_network().max_vss_on_edge(e);
    }

    for (const size_t tr :
         m_instance.all_trains_on_edge(e, this->fix_routes, false)) {
      const auto& tr_name =
          m_instance.get_const_train_list().get_train(tr).get_name();
      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        GRBLinExpr lhs = m_vars["e_tight"](tr, t, e);
        if (breakable_e_index.has_value()) {
          for (size_t vss = 0; vss < vss_e.value(); ++vss) {
            lhs += m_vars["b_tight"](tr, t, breakable_e_index.value(), vss);
          }
        }
        m_model->addConstr(lhs, GRB_GREATER_EQUAL,
                           m_vars["x"](tr, t - 1, e) - m_vars["stopped"](tr, t),
                           "b_tight_e_tight_min_one_" + tr_name + "_" +
                               std::to_string(t * dt) + "_" + edge_name);
      }
    }
  }

  // On every edge that is not breakable and does not end with a border at least
  // one out edge has to be used if it is used and v = 0
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name =
        m_instance.get_const_train_list().get_train(tr).get_name();
    const auto& edge_used_tr =
        m_instance.edges_used_by_train(tr, this->fix_routes);
    for (const size_t e : edge_used_tr) {
      const auto& edge = m_instance.get_const_network().get_edge(e);
      const auto& edge_name =
          "[" + m_instance.get_const_network().get_vertex(edge.source).name +
          "," + m_instance.get_const_network().get_vertex(edge.target).name +
          "]";
      if (edge.breakable ||
          m_instance.get_const_network().get_vertex(edge.target).type !=
              VertexType::NoBorder) {
        continue;
      }

      const auto& delta_out = m_instance.get_const_network().get_successors(e);
      cda_rail::index_vector delta_out_tr;
      for (const auto& e_out : delta_out) {
        if (std::ranges::contains(edge_used_tr, e_out)) {
          delta_out_tr.emplace_back(e_out);
        }
      }

      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        GRBLinExpr lhs = 0;
        for (const auto& e_out : delta_out_tr) {
          lhs += m_vars["x"](tr, t - 1, e_out);
        }
        m_model->addConstr(lhs, GRB_GREATER_EQUAL,
                           m_vars["x"](tr, t - 1, e) - m_vars["stopped"](tr, t),
                           "no_stop_on_non-border_edge_ending_" + tr_name +
                               "_" + std::to_string(t * dt) + "_" + edge_name);
      }
    }
  }

  // b cannot be tight if it is not front. If v = 0 then it has to be
  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e    = breakable_edges[i];
    const auto& edge = m_instance.get_const_network().get_edge(e);
    const auto& edge_name =
        "[" + m_instance.get_const_network().get_vertex(edge.source).name +
        "," + m_instance.get_const_network().get_vertex(edge.target).name + "]";
    const auto& vss_e = m_instance.get_const_network().max_vss_on_edge(e);
    for (const size_t tr :
         m_instance.all_trains_on_edge(e, this->fix_routes, false)) {
      const auto& tr_name =
          m_instance.get_const_train_list().get_train(tr).get_name();
      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        for (size_t vss = 0; vss < vss_e; ++vss) {
          m_model->addConstr(m_vars["b_tight"](tr, t, i, vss), GRB_LESS_EQUAL,
                             m_vars["b_front"](tr, t, i, vss),
                             "b_tight_not_front_1_" + tr_name + "_" +
                                 std::to_string(t * dt) + "_" + edge_name +
                                 "_" + std::to_string(vss));
          m_model->addConstr(
              m_vars["b_tight"](tr, t, i, vss), GRB_GREATER_EQUAL,
              m_vars["b_front"](tr, t, i, vss) - m_vars["stopped"](tr, t),
              "b_tight_not_front_2_" + tr_name + "_" + std::to_string(t * dt) +
                  "_" + edge_name + "_" + std::to_string(vss));
        }
      }
    }
  }

  // At least any one tight if speed is 0
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name =
        m_instance.get_const_train_list().get_train(tr).get_name();
    const auto& edge_used_tr =
        m_instance.edges_used_by_train(tr, this->fix_routes);
    for (size_t t = train_interval[tr].first + 2;
         t <= train_interval[tr].second; ++t) {
      GRBLinExpr lhs = 0;
      for (const size_t e : edge_used_tr) {
        lhs += m_vars["e_tight"](tr, t, e);
        const auto& edge = m_instance.get_const_network().get_edge(e);
        if (!edge.breakable) {
          continue;
        }
        const auto& vss_e = m_instance.get_const_network().max_vss_on_edge(e);
        for (size_t vss = 0; vss < vss_e; ++vss) {
          lhs += m_vars["b_tight"](tr, t, breakable_edge_indices.at(e), vss);
        }
      }
      m_model->addConstr(lhs, GRB_GREATER_EQUAL, 1 - m_vars["stopped"](tr, t),
                         "at_least_one_tight_if_stopped_" + tr_name + "_" +
                             std::to_string(t * dt));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_variables() {
  PLOGD << "Create general variables";
  create_general_variables();
  if (this->fix_routes) {
    PLOGD << "Create fixed routes variables";
    create_fixed_routes_variables();
  } else {
    PLOGD << "Create free routes variables";
    create_free_routes_variables();
  }
  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    PLOGD << "Create discretized VSS variables";
    create_discretized_variables();
  } else {
    PLOGD << "Create non-discretized VSS variables";
    create_non_discretized_variables();
  }
  if (this->include_braking_curves) {
    PLOGD << "Create braking distance variables";
    create_brakelen_variables();
  }
  if (vss_model.get_only_stop_at_vss()) {
    PLOGD << "Create only stop at VSS variables";
    create_only_stop_at_vss_variables();
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_constraints() {
  PLOGD << "Create general constraints";
  create_general_constraints();
  if (this->fix_routes) {
    PLOGD << "Create fixed routes constraints";
    create_fixed_routes_constraints();
  } else {
    PLOGD << "Create free routes constraints";
    create_free_routes_constraints();
  }
  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    PLOGD << "Create discretized VSS constraints";
    create_discretized_constraints();
  } else {
    PLOGD << "Create non-discretized VSS constraints";
    create_non_discretized_constraints();
  }
  if (this->include_train_dynamics) {
    PLOGD << "Create train dynamic constraints";
    create_acceleration_constraints();
  }
  if (this->include_braking_curves) {
    PLOGD << "Create braking distance constraints";
    create_brakelen_constraints();
  }
}

// NOLINTEND(performance-inefficient-string-concatenation)

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-unchecked-optional-access)
