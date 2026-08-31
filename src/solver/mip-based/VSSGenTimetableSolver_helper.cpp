#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <plog/Log.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using std::size_t;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-unchecked-optional-access)

cda_rail::index_vector
cda_rail::solver::mip_based::VSSGenTimetableSolver::unbreakable_section_indices(
    size_t train_index) const {
  /**
   * This function returns the indices of the unbreakable sections that are
   * traversed by the train with index train_index
   * @param train_index index of the train
   * @return vector of indices
   */

  cda_rail::index_vector indices;
  const auto&            tr_name =
      m_instance.get_const_train_list().get_train(train_index).get_name();
  const auto& tr_route =
      m_instance.get_const_routes().get_route(tr_name).get_edges();
  for (size_t i = 0; i < unbreakable_sections.size(); ++i) {
    bool edge_found = false;
    // If unbreakable_section[i] (of type vector) and tr_route (of type vector)
    // overlap (have a common element), add i to indices
    for (size_t j0 = 0; j0 < unbreakable_sections[i].size() && !edge_found;
         ++j0) {
      for (size_t j1 = 0; j1 < tr_route.size() && !edge_found; ++j1) {
        if (unbreakable_sections[i][j0] == tr_route[j1]) {
          indices.push_back(i);
          edge_found = true;
        }
      }
    }
  }

  return indices;
}

cda_rail::solver::mip_based::VSSGenTimetableSolver::TemporaryImpossibilityStruct
cda_rail::solver::mip_based::VSSGenTimetableSolver::
    get_temporary_impossibility_struct(const size_t& tr,
                                       const size_t& t) const {
  /**
   * This returns a struct containing information about the previous and
   * following station.
   *
   * @param tr index of the train
   * @param t time index
   *
   * @return struct containing information about the previous and following
   * station
   */

  // Initialize struct
  TemporaryImpossibilityStruct s;

  const auto& train_list  = m_instance.get_const_train_list();
  const auto  tr_name     = train_list.get_train(tr).get_name();
  const auto& tr_schedule = m_instance.get_const_schedule(tr_name);

  s.to_use   = true;
  s.t_before = train_interval[tr].first;
  s.t_after  = train_interval[tr].second + 1;
  s.v_before = tr_schedule.get_initial_velocity();
  s.v_after  = tr_schedule.get_exit_velocity();

  for (const auto& tr_stop : tr_schedule.get_stops()) {
    const auto t0 = tr_stop.get_service_time() / dt;
    const auto t1 = static_cast<int>(
        std::ceil(static_cast<double>(tr_stop.get_earliest_departure()) / dt));
    if (t >= t0 && t <= t1) {
      s.to_use = false;
      return s;
    }
    if (t0 < t && t0 > s.t_before) {
      s.t_before = t0;
      s.edges_before =
          cda_rail::index_vector(tr_stop.get_station().tracks.begin(),
                                 tr_stop.get_station().tracks.end());
      s.v_before = 0;
    }
    if (t1 > t && t1 < s.t_after) {
      s.t_after = t1;
      s.edges_after =
          cda_rail::index_vector(tr_stop.get_station().tracks.begin(),
                                 tr_stop.get_station().tracks.end());
      s.v_after = 0;
    }
  }

  return s;
}

double
cda_rail::solver::mip_based::VSSGenTimetableSolver::max_distance_travelled(
    const size_t& tr, const size_t& time_steps, const double& v0,
    const double& a_max, const bool& braking_distance) const {
  const auto& train_object = m_instance.get_const_train_list().get_train(tr);
  const auto  v_max        = train_object.get_max_speed();
  const auto  time_diff    = static_cast<int>(time_steps) * dt;
  double      ret_val      = 0;
  double      final_speed  = NAN;
  if (!this->include_train_dynamics) {
    ret_val += time_diff * v_max;
    final_speed = v_max;
  } else if (time_diff < (v_max - v0) / a_max) {
    ret_val +=
        0.5 * time_diff *
        (a_max * time_diff + 2 * v0); // int_{0}^{time_diff} (a_max*t + v0) dt
    final_speed = a_max * time_diff + v0;
  } else {
    ret_val += (v_max - v0) * (v_max + v0) /
               (2 * a_max); // int_{0}^{(v_max-v0)/a_max} (a_max*t + v0) dt
    ret_val += (time_diff - (v_max - v0) / a_max) *
               v_max; // int_{(v_max-v0)/a_max}^{time_diff} v_max dt
    final_speed = v_max;
  }
  if (braking_distance) {
    ret_val +=
        final_speed * final_speed / (2 * train_object.get_deceleration());
  }
  return ret_val;
}

cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance
cda_rail::solver::mip_based::VSSGenTimetableSolver::extract_solution(
    bool postprocess, bool full_model,
    const std::optional<instances::GeneralPerformanceOptimizationInstance>&
        old_instance) const {
  PLOGD << "Extracting solution object...";

  auto sol_obj = instances::SolVSSGeneralPerformanceOptimizationInstance(
      (old_instance.has_value() ? old_instance.value() : m_instance));

  if (const auto grb_status = m_model->get(GRB_IntAttr_Status);
      full_model && grb_status == GRB_OPTIMAL) {
    PLOGD << "Solution status: Optimal";
    sol_obj.set_status(SolutionStatus::Optimal);
  } else if (grb_status == GRB_INFEASIBLE) {
    PLOGD << "Solution status: Infeasible";
    sol_obj.set_status(SolutionStatus::Infeasible);
  } else if (m_model->get(GRB_IntAttr_SolCount) >= 1) {
    PLOGD << "Solution status: Feasible (optimality unknown)";
    sol_obj.set_status(SolutionStatus::Feasible);
  } else if (grb_status == GRB_TIME_LIMIT &&
             m_model->get(GRB_IntAttr_SolCount) == 0) {
    PLOGD << "Solution status: Timeout (Feasibility unknown)";
    sol_obj.set_status(SolutionStatus::Timeout);
  } else {
    PLOGE << "Solution status code " << grb_status << " unknown";
    throw exceptions::ConsistencyException(
        "Gurobi status code " + std::to_string(grb_status) + " unknown.");
  }

  if (const auto sol_count = m_model->get(GRB_IntAttr_SolCount);
      sol_count < 0.5) {
    return sol_obj;
  }

  const auto mip_obj_val =
      static_cast<int>(std::round(m_model->get(GRB_DoubleAttr_ObjVal)));
  PLOGD << "MIP objective: " << mip_obj_val;

  if (vss_model.get_model_type() == vss::ModelType::Discrete) {
    // TODO: Implement
    sol_obj.set_obj(mip_obj_val);
    return sol_obj;
  }

  sol_obj.set_solution_found();

  int obj = 0;

  for (size_t r_e_index = 0; r_e_index < relevant_edges.size(); ++r_e_index) {
    const auto e_index = relevant_edges.at(r_e_index);
    const auto vss_number_e =
        m_instance.get_const_network().max_vss_on_edge(e_index);
    const auto& e = m_instance.get_const_network().get_edge(e_index);
    const auto  reverse_edge_index =
        m_instance.get_const_network().get_reverse_edge_index(e_index);
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      bool b_used = false;

      if (vss_model.get_model_type() == vss::ModelType::Continuous) {
        b_used =
            m_vars.at("b_used").at(r_e_index, vss).get(GRB_DoubleAttr_X) > 0.5;
      } else if (vss_model.get_model_type() == vss::ModelType::Inferred) {
        b_used =
            m_vars.at("num_vss_segments").at(r_e_index).get(GRB_DoubleAttr_X) >
            static_cast<double>(vss) + 1.5;
      } else if (vss_model.get_model_type() == vss::ModelType::InferredAlt) {
        // if any of "type_num_vss_segments"(r_e_index, sep_type_index, num_vss)
        // is > 0.5 for vss <= num_vss < vss_number_e for sep_type_index = 0,
        // ..., num_sep_types - 1 then b_used = true
        for (size_t sep_type_index = 0;
             sep_type_index < vss_model.get_separation_functions().size();
             ++sep_type_index) {
          for (size_t num_vss = vss; num_vss < vss_number_e; ++num_vss) {
            if (m_vars.at("type_num_vss_segments")
                    .at(r_e_index, sep_type_index, num_vss)
                    .get(GRB_DoubleAttr_X) > 0.5) {
              b_used = true;
              break;
            }
          }
          if (b_used) {
            break;
          }
        }
      }

      if (postprocess && b_used) {
        IF_PLOG(plog::debug) {
          const auto& source =
              m_instance.get_const_network().get_vertex(e.source).name;
          const auto& target =
              m_instance.get_const_network().get_vertex(e.target).name;
          PLOGD << "Postprocessing on " << source << " to " << target;
        }
        b_used = false;
        for (size_t tr = 0; tr < num_tr; ++tr) {
          for (size_t t = train_interval.at(tr).first;
               t <= train_interval.at(tr).second; ++t) {
            const auto front1 =
                m_vars.at("b_front")
                    .at(tr, t, breakable_edge_indices.at(e_index), vss)
                    .get(GRB_DoubleAttr_X) > 0.5;
            const auto rear1 =
                m_vars.at("b_rear")
                    .at(tr, t, breakable_edge_indices.at(e_index), vss)
                    .get(GRB_DoubleAttr_X) > 0.5;
            const auto front2 =
                (reverse_edge_index.has_value() &&
                 !m_instance
                      .trains_on_edge(reverse_edge_index.value(), fix_routes,
                                      {tr})
                      .empty())
                    ? m_vars.at("b_front")
                              .at(tr, t,
                                  breakable_edge_indices.at(
                                      reverse_edge_index.value()),
                                  vss)
                              .get(GRB_DoubleAttr_X) > 0.5
                    : false;
            const auto rear2 =
                (reverse_edge_index.has_value() &&
                 !m_instance
                      .trains_on_edge(reverse_edge_index.value(), fix_routes,
                                      {tr})
                      .empty())
                    ? m_vars.at("b_rear")
                              .at(tr, t,
                                  breakable_edge_indices.at(
                                      reverse_edge_index.value()),
                                  vss)
                              .get(GRB_DoubleAttr_X) > 0.5
                    : false;
            if (front1 || rear1 || front2 || rear2) {
              b_used = true;
              break;
            }
          }
          if (b_used) {
            break;
          }
        }
      }

      if (!b_used) {
        continue;
      }

      const auto b_pos_val = round_to_given_tolerance(
          m_vars.at("b_pos")
              .at(breakable_edge_indices.at(e_index), vss)
              .get(GRB_DoubleAttr_X),
          ROUNDING_PRECISION);
      IF_PLOG(plog::debug) {
        const auto& source =
            m_instance.get_const_network().get_vertex(e.source).name;
        const auto& target =
            m_instance.get_const_network().get_vertex(e.target).name;
        PLOGD << "Add VSS at " << b_pos_val << " on " << source << " to "
              << target;
      }
      sol_obj.add_vss_pos(e_index, b_pos_val, true);
      obj += 1;
    }
  }

  sol_obj.set_obj(obj);

  if (!fix_routes) {
    sol_obj.reset_routes();
    PLOGD << "Extracting routes";
    for (size_t tr = 0; tr < num_tr; ++tr) {
      const auto train = m_instance.get_const_train_list().get_train(tr);
      sol_obj.add_empty_route(train.get_name());
      size_t current_vertex =
          m_instance.get_const_schedule(tr).get_entry_vertex();
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        std::unordered_set<size_t> edge_list;
        for (int e = 0; e < num_edges; ++e) {
          const auto tr_on_edge =
              m_vars.at("x").at(tr, t, e).get(GRB_DoubleAttr_X) > 0.5;
          if (tr_on_edge &&
              !sol_obj.get_const_solution_routes()
                   .get_route(train.get_name())
                   .contains_edge(e) &&
              !edge_list.contains(e)) {
            edge_list.emplace(e);
          }
        }
        while (!edge_list.empty()) {
          bool edge_added = false;
          for (const auto& e : edge_list) {
            if (m_instance.get_const_network().get_edge(e).source ==
                current_vertex) {
              sol_obj.push_back_edge_to_route(train.get_name(), e);
              current_vertex =
                  m_instance.get_const_network().get_edge(e).target;
              edge_list.erase(e);
              edge_added = true;
              break;
            }
          }
          if (!edge_added) {
            throw exceptions::ConsistencyException("Error in route extraction");
          }
        }
      }
    }
  }

  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto train = m_instance.get_const_train_list().get_train(tr);
    for (size_t t = train_interval[tr].first;
         t <= train_interval[tr].second + 1; ++t) {
      auto const& tr_name         = sol_obj.get_instance()
                                        ->get_const_train_list()
                                        .get_train(tr)
                                        .get_name();
      const auto  train_speed_val = round_to_given_tolerance(
          m_vars.at("v").at(tr, t).get(GRB_DoubleAttr_X), V_MIN);
      sol_obj.add_train_speed(tr_name, static_cast<double>(t) * dt,
                              train_speed_val);
    }
  }

  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto  train  = m_instance.get_const_train_list().get_train(tr);
    const auto& tr_len = train.get_length();
    const auto& r_len  = sol_obj.route_length(train.get_name());
    for (auto t = train_interval[tr].first; t <= train_interval[tr].second;
         ++t) {
      double train_pos = r_len;
      if (fix_routes) {
        train_pos = m_vars.at("lda").at(tr, t).get(GRB_DoubleAttr_X);
      } else {
        const double len_in = round_to_given_tolerance(
            m_vars.at("len_in").at(tr, t).get(GRB_DoubleAttr_X),
            ROUNDING_PRECISION);
        if (len_in > EPS) {
          train_pos = -len_in;
        } else {
          for (auto e_index : sol_obj.get_const_solution_routes()
                                  .get_route(train.get_name())
                                  .get_edges()) {
            const bool e_used =
                m_vars.at("x").at(tr, t, e_index).get(GRB_DoubleAttr_X) > 0.5;
            if (e_used) {
              const double lda_val =
                  m_vars.at("e_lda").at(tr, t, e_index).get(GRB_DoubleAttr_X);
              const double e_pos =
                  sol_obj.get_const_solution_routes()
                      .route_edge_pos(train.get_name(), e_index)
                      .first;
              train_pos = std::min(lda_val + e_pos, train_pos);
            }
          }
        }
      }

      train_pos += tr_len;
      train_pos = round_to_given_tolerance(train_pos, ROUNDING_PRECISION);
      sol_obj.add_train_pos(train.get_name(), static_cast<double>(t) * dt,
                            train_pos);
    }

    auto   t_final         = train_interval[tr].second + 1;
    double train_pos_final = NAN;
    if (fix_routes) {
      train_pos_final = round_to_given_tolerance(
          m_vars.at("mu").at(tr, t_final - 1).get(GRB_DoubleAttr_X),
          ROUNDING_PRECISION);
    } else {
      train_pos_final =
          r_len +
          round_to_given_tolerance(
              m_vars.at("len_out").at(tr, t_final - 1).get(GRB_DoubleAttr_X),
              ROUNDING_PRECISION);
    }
    if (include_braking_curves) {
      train_pos_final -= round_to_given_tolerance(
          m_vars.at("brakelen").at(tr, t_final - 1).get(GRB_DoubleAttr_X),
          ROUNDING_PRECISION);
    }
    train_pos_final =
        round_to_given_tolerance(train_pos_final, ROUNDING_PRECISION);
    sol_obj.add_train_pos(train.get_name(), static_cast<double>(t_final) * dt,
                          train_pos_final);
  }

  return sol_obj;
}

std::pair<std::vector<cda_rail::index_vector>,
          std::vector<cda_rail::index_vector>>
cda_rail::solver::mip_based::VSSGenTimetableSolver::common_entry_exit_vertices()
    const {
  /**
   * Returns trains that have common entry or exit vertices sorted by entry/exit
   * time
   */

  auto compare_entry = [this](size_t tr1, size_t tr2) {
    return train_interval[tr1].first < train_interval[tr2].first;
  };
  auto compare_exit = [this](size_t tr1, size_t tr2) {
    return train_interval[tr1].second > train_interval[tr2].second;
  };

  std::pair<std::vector<cda_rail::index_vector>,
            std::vector<cda_rail::index_vector>>
                                                     ret_val;
  std::unordered_map<size_t, cda_rail::index_vector> entry_vertices;
  std::unordered_map<size_t, cda_rail::index_vector> exit_vertices;

  for (size_t tr = 0; tr < num_tr; ++tr) {
    entry_vertices[m_instance.get_const_schedule(tr).get_entry_vertex()]
        .push_back(tr);
    exit_vertices[m_instance.get_const_schedule(tr).get_exit_vertex()]
        .push_back(tr);
  }

  for (auto& [_, tr_list] : entry_vertices) {
    if (tr_list.size() > 1) {
      std::sort(tr_list.begin(), tr_list.end(), compare_entry);
      ret_val.first.push_back(tr_list);
    }
  }
  for (auto& [_, tr_list] : exit_vertices) {
    if (tr_list.size() > 1) {
      std::sort(tr_list.begin(), tr_list.end(), compare_exit);
      ret_val.second.push_back(tr_list);
    }
  }

  return ret_val;
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::cleanup() {
  dt                     = -1;
  num_t                  = 0;
  num_tr                 = 0;
  num_edges              = 0;
  num_vertices           = 0;
  num_breakable_sections = 0;
  unbreakable_sections.clear();
  no_border_vss_sections.clear();
  train_interval.clear();
  breakable_edges_pairs.clear();
  no_border_vss_vertices.clear();
  relevant_edges.clear();
  breakable_edges.clear();
  fix_routes                = false;
  vss_model                 = vss::Model();
  include_train_dynamics    = false;
  use_pwl                   = false;
  use_schedule_cuts         = false;
  export_option             = ExportOption::NoExport;
  iterative_vss             = false;
  optimality_strategy       = OptimalityStrategy::Optimal;
  iterative_update_strategy = UpdateStrategy::Fixed;
  iterative_initial_value   = 1;
  iterative_update_value    = 2;
  iterative_include_cuts    = true;
  postprocess               = false;
  max_vss_per_edge_in_iteration.clear();
  breakable_edge_indices.clear();
  fwd_bwd_sections.clear();
  GeneralMIPSolver::cleanup();
}

bool cda_rail::solver::mip_based::VSSGenTimetableSolver::update_vss(
    size_t relevant_edge_index, double obj_ub, GRBLinExpr& cut_expr) {
  const auto& e = relevant_edges.at(relevant_edge_index);
  const auto  vss_number_e =
      m_instance.get_editable_network().max_vss_on_edge(e);
  const auto& current_vss_number_e =
      max_vss_per_edge_in_iteration.at(relevant_edge_index);

  size_t increase_val = 1;
  if (iterative_update_strategy == UpdateStrategy::Fixed) {
    increase_val =
        std::max(increase_val, static_cast<size_t>(std::ceil(
                                   (iterative_update_value - 1) *
                                   static_cast<double>(current_vss_number_e))));
  } else if (iterative_update_strategy == UpdateStrategy::Relative) {
    increase_val = std::max(
        increase_val,
        static_cast<size_t>(std::ceil(iterative_update_value *
                                      static_cast<double>(vss_number_e))));
  }

  auto target_vss_number_e = (m_model->get(GRB_IntAttr_SolCount) >= 1)
                                 ? static_cast<size_t>(std::round(obj_ub - 1))
                                 : current_vss_number_e + increase_val;

  target_vss_number_e = std::min<uint64_t>(target_vss_number_e, vss_number_e);
  if (target_vss_number_e <= current_vss_number_e) {
    return false;
  }

  update_max_vss_on_edge(relevant_edge_index, target_vss_number_e, cut_expr);
  return true;
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::update_max_vss_on_edge(
    size_t relevant_edge_index, size_t new_max_vss, GRBLinExpr& cut_expr) {
  const auto& e = relevant_edges.at(relevant_edge_index);
  const auto  vss_number_e =
      m_instance.get_editable_network().max_vss_on_edge(e);
  const auto old_max_vss =
      max_vss_per_edge_in_iteration.at(relevant_edge_index);
  max_vss_per_edge_in_iteration[relevant_edge_index] = new_max_vss;

  IF_PLOG(plog::debug) {
    const auto& u =
        m_instance.get_editable_network()
            .get_vertex(m_instance.get_editable_network().get_edge(e).source)
            .name;
    const auto& v =
        m_instance.get_editable_network()
            .get_vertex(m_instance.get_editable_network().get_edge(e).target)
            .name;
    PLOGD << "Update possible VSS on edge " << u << " -> " << v << " from "
          << old_max_vss << " to " << new_max_vss;
  }

  if (this->vss_model.get_model_type() == vss::ModelType::Inferred) {
    m_vars.at("num_vss_segments")(relevant_edge_index)
        .set(GRB_DoubleAttr_UB, static_cast<double>(new_max_vss) + 1);
    if (this->iterative_include_cuts_tmp && new_max_vss > old_max_vss) {
      const auto b =
          m_model->addVar(0, 1, 0, GRB_BINARY,
                          "binary_cut_" + std::to_string(relevant_edge_index) +
                              "_" + std::to_string(old_max_vss));
      // b = 1 iff num_vss_segments(relevant_edge_index) >= old_max_vss + 1
      m_model->addConstr(m_vars.at("num_vss_segments")(relevant_edge_index) -
                                 static_cast<double>(old_max_vss) <=
                             (vss_number_e + 1) * b,
                         "binary_cut_relation_" +
                             std::to_string(relevant_edge_index) + "_" +
                             std::to_string(old_max_vss) + "_1");
      m_model->addConstr(
          static_cast<double>(old_max_vss + 1) -
                  m_vars.at("num_vss_segments")(relevant_edge_index) <=
              (vss_number_e) * (1 - b),
          "binary_cut_relation_" + std::to_string(relevant_edge_index) + "_" +
              std::to_string(old_max_vss) + "_2");
      cut_expr += b;
      PLOGD << "Add binary_cut_" << relevant_edge_index << "_" << old_max_vss
            << "to cut_expr";
    }
  }
  if (this->vss_model.get_model_type() == vss::ModelType::Continuous) {
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      m_vars.at("b_used")(relevant_edge_index, vss)
          .set(GRB_DoubleAttr_UB, static_cast<double>(vss < new_max_vss));
    }
    if (this->iterative_include_cuts_tmp && new_max_vss > old_max_vss) {
      cut_expr += m_vars.at("b_used")(relevant_edge_index, old_max_vss);
      PLOGD << "Add b_used(" << relevant_edge_index << "," << old_max_vss
            << ") to cut_expr";
    }
  }
  if (this->vss_model.get_model_type() == vss::ModelType::InferredAlt) {
    for (size_t sep_type = 0;
         sep_type < this->vss_model.get_separation_functions().size();
         ++sep_type) {
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        m_vars.at("type_num_vss_segments")(relevant_edge_index, sep_type, vss)
            .set(GRB_DoubleAttr_UB, static_cast<double>(vss < new_max_vss));
      }
      if (this->iterative_include_cuts_tmp && new_max_vss > old_max_vss) {
        cut_expr += m_vars.at("type_num_vss_segments")(relevant_edge_index,
                                                       sep_type, old_max_vss);
        PLOGD << "Add type_num_vss_segments(" << relevant_edge_index << ","
              << sep_type << "," << old_max_vss << ") to cut_expr";
      }
    }
  }
}

std::optional<cda_rail::instances::GeneralPerformanceOptimizationInstance>
cda_rail::solver::mip_based::VSSGenTimetableSolver::initialize_variables(
    const cda_rail::solver::mip_based::ModelDetail&      model_detail,
    const cda_rail::solver::mip_based::ModelSettings&    model_settings,
    const cda_rail::solver::mip_based::SolverStrategy&   solver_strategy,
    const cda_rail::solver::mip_based::SolutionSettings& solution_settings,
    int time_limit, bool debug_input, bool overwrite_severity) {
  /**
   * This function initializes the variables affecting the m_model creation and
   * optimization process
   */
  this->solve_init_vss_gen_timetable(debug_input, overwrite_severity);

  if (!model_settings.model_type.check_consistency()) {
    PLOGE << "Model type  and separation types/functions are not consistent.";
    throw cda_rail::exceptions::ConsistencyException(
        "Model type and separation types/functions are not consistent.");
  }

  if (!m_instance.get_editable_network().is_consistent_for_transformation()) {
    PLOGE << "Instance is not consistent for transformation.";
    throw exceptions::ConsistencyException();
  }

  // General settings
  this->dt                        = model_detail.delta_t;
  this->fix_routes                = model_detail.fix_routes;
  this->vss_model                 = model_settings.model_type;
  this->include_train_dynamics    = model_detail.train_dynamics;
  this->include_braking_curves    = model_detail.braking_curves;
  this->use_pwl                   = model_settings.use_pwl;
  this->use_schedule_cuts         = model_settings.use_schedule_cuts;
  this->iterative_vss             = solver_strategy.iterative_approach;
  this->optimality_strategy       = solver_strategy.optimality_strategy;
  this->iterative_update_strategy = solver_strategy.update_strategy;
  this->iterative_initial_value   = solver_strategy.initial_value;
  this->iterative_update_value    = solver_strategy.update_value;
  this->iterative_include_cuts    = solver_strategy.include_cuts;
  this->postprocess               = solution_settings.postprocess;
  this->export_option             = solution_settings.export_option;

  if (this->iterative_vss) {
    // Iterative optimization strategy
    if (this->iterative_update_strategy == UpdateStrategy::Fixed &&
        this->iterative_update_value <= 1) {
      PLOGE << "iterative_update_value must be greater than 1";
      throw exceptions::ConsistencyException(
          "iterative_update_value must be greater than 1");
    }
    if (this->iterative_update_strategy == UpdateStrategy::Relative &&
        (this->iterative_update_value <= 0 ||
         this->iterative_update_value >= 1)) {
      PLOGE << "iterative_update_value must be between 0 and 1";
      throw exceptions::ConsistencyException(
          "iterative_update_value must be between 0 and 1");
    }
  }

  if (this->fix_routes && !m_instance.has_route_for_every_train()) {
    PLOGE << "Instance does not have a route for every train";
    throw exceptions::ConsistencyException(
        "Instance does not have a route for every train");
  }

  std::optional<instances::GeneralPerformanceOptimizationInstance> old_instance;
  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    // Discretize graph m_model
    PLOGI << "Preprocessing graph...";
    old_instance = m_instance;
    m_instance.discretize(this->vss_model.get_separation_functions().front());
    PLOGI << "Preprocessing graph... DONE";
  }

  PLOGI << "Creating m_model...";
  PLOGD << "Initialize other relevant variables";

  num_t = static_cast<size_t>(
      m_instance.get_const_timetable().latest_exit_time() / dt);
  if (std::fmod(m_instance.get_const_timetable().latest_exit_time(), dt) !=
      0.0) {
    num_t += 1;
  }

  num_tr       = m_instance.get_const_train_list().size();
  num_edges    = m_instance.get_editable_network().number_of_edges();
  num_vertices = m_instance.get_editable_network().number_of_vertices();

  {
    const auto raw_unbreakable =
        m_instance.get_editable_network().unbreakable_sections();
    unbreakable_sections.clear();
    for (const auto& section : raw_unbreakable) {
      unbreakable_sections.emplace_back(section.begin(), section.end());
    }
  }

  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    // Sections of discretized graph
    const auto raw_no_border =
        m_instance.get_editable_network().no_border_vss_sections();
    no_border_vss_sections.clear();
    for (const auto& section : raw_no_border) {
      no_border_vss_sections.emplace_back(section.begin(), section.end());
    }
    num_breakable_sections = no_border_vss_sections.size();
    no_border_vss_vertices =
        m_instance.get_editable_network().get_vertices_by_type(
            VertexType::NoBorderVSS);
  } else {
    // Sections of non-discretized graph
    breakable_edges = m_instance.get_editable_network().breakable_edges();
    for (size_t i = 0; i < breakable_edges.size(); ++i) {
      breakable_edge_indices[breakable_edges[i]] = i;
    }
    breakable_edges_pairs =
        m_instance.get_editable_network().combine_reverse_edges(
            breakable_edges);
    num_breakable_sections = breakable_edges.size();
    relevant_edges =
        m_instance.get_editable_network().relevant_breakable_edges();
  }

  for (size_t i = 0; i < num_tr; ++i) {
    train_interval.emplace_back(
        m_instance.get_const_timetable().time_index_interval(i, dt, false));
  }

  if (iterative_vss && vss_model.get_model_type() == vss::ModelType::Discrete) {
    PLOGE << "Iterative VSS not supported for discrete VSS m_model";
    throw exceptions::ConsistencyException(
        "Iterative VSS not supported for discrete VSS m_model");
  }

  max_vss_per_edge_in_iteration.resize(relevant_edges.size(), 0);
  for (size_t i = 0; i < relevant_edges.size(); ++i) {
    const auto& e = relevant_edges.at(i);
    const auto  vss_number_e =
        m_instance.get_editable_network().max_vss_on_edge(e);
    if (iterative_vss) {
      if (iterative_update_strategy == UpdateStrategy::Fixed) {
        max_vss_per_edge_in_iteration[i] = std::min<size_t>(
            vss_number_e,
            static_cast<size_t>(std::ceil(iterative_initial_value)));
      } else if (iterative_update_strategy == UpdateStrategy::Relative) {
        max_vss_per_edge_in_iteration[i] = std::min<size_t>(
            vss_number_e,
            static_cast<size_t>(std::ceil(iterative_initial_value *
                                          static_cast<double>(vss_number_e))));
      } else {
        PLOGE << "Unknown update strategy";
        throw exceptions::ConsistencyException("Unknown update strategy");
      }
    } else {
      max_vss_per_edge_in_iteration[i] = vss_number_e;
    }
  }

  // Sections on which trains can collide face-to-face
  calculate_fwd_bwd_sections();

  // Return unchanged m_instance for cleaning step
  return old_instance;
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::set_timeout(
    int time_limit) {
  PLOGI << "DONE creating m_model";

  if (plog::get()->checkSeverity(plog::debug) || time_limit > 0) {
    m_model_created = std::chrono::high_resolution_clock::now();
    m_create_time   = std::chrono::duration_cast<std::chrono::milliseconds>(
                          m_model_created - m_start)
                          .count();

    auto time_left = time_limit - (m_create_time / 1000);
    if (time_left < 0 && time_limit > 0) {
      time_left = 1;
    }
    if (time_limit > 0) {
      m_model->set(GRB_DoubleParam_TimeLimit, static_cast<double>(time_left));
    }
    PLOGD << "Model created in "
          << (static_cast<double>(m_create_time) / 1000.0) << " s";
    if (time_limit > 0) {
      PLOGD << "Time left: " << time_left << " s";
    } else {
      PLOGD << "Time left: "
            << "No Limit";
    }
  }

  if ((this->include_braking_curves && !this->use_pwl)) {
    // Non-convex constraints are present. Still, Gurobi can solve to optimality
    // using spatial branching
    m_model->set(GRB_IntParam_NonConvex, 2);
  }
}

std::optional<cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance>
cda_rail::solver::mip_based::VSSGenTimetableSolver::optimize(
    const std::optional<instances::GeneralPerformanceOptimizationInstance>&
        old_instance,
    int time_limit) {
  /**
   * This function contains the optimization process
   */
  std::optional<instances::SolVSSGeneralPerformanceOptimizationInstance>
      sol_object;

  bool reoptimize = true;

  double obj_ub = 1.0;
  for (const auto& e : relevant_edges) {
    obj_ub += m_instance.get_const_network().max_vss_on_edge(e);
  }
  double obj_lb           = 0;
  size_t iteration_number = 0;

  std::vector<GRBConstr> iterative_cuts;
  this->iterative_include_cuts_tmp = this->iterative_include_cuts;

  while (reoptimize) {
    reoptimize = false;

    if (optimality_strategy == OptimalityStrategy::Feasible) {
      m_model->set(GRB_IntParam_SolutionLimit, 1);
      m_model->set(GRB_IntParam_MIPFocus, 1);
      PLOGD << "Settings focussing on feasibility";
    }

    // Optimize the (possibly restricted) m_model
    this->m_model->optimize();
    iteration_number += 1;

    if (m_model->get(GRB_IntAttr_SolCount) >= 1) {
      // If there is a solution, then extract it and compare it with the current
      // best solution
      const auto obj_tmp = m_model->get(GRB_DoubleAttr_ObjVal);
      if (obj_tmp < obj_ub) {
        obj_ub = obj_tmp;
        sol_object =
            extract_solution(postprocess, !iterative_vss, old_instance);
        this->iterative_include_cuts_tmp = false;
      }
    }

    if (!sol_object.has_value()) {
      sol_object = extract_solution(postprocess, !iterative_vss, old_instance);
    }

    if (iterative_vss) {
      // If applicable, iteratively update the m_model
      if (m_model->get(GRB_IntAttr_Status) == GRB_TIME_LIMIT) {
        PLOGD << "Break because of timeout";
        if (sol_object->has_solution()) {
          PLOGD << "However, use previous obtained solution";
          break;
        }
        sol_object =
            extract_solution(postprocess, !iterative_vss, old_instance);
        break;
      }

      auto obj_lb_tmp = m_model->get(GRB_DoubleAttr_ObjBound);
      for (int i = 0; i < relevant_edges.size(); ++i) {
        if (static_cast<double>(max_vss_per_edge_in_iteration.at(i)) + 1 <
                obj_lb_tmp &&
            max_vss_per_edge_in_iteration.at(i) <
                m_instance.get_const_network().max_vss_on_edge(
                    relevant_edges.at(i))) {
          obj_lb_tmp =
              static_cast<double>(max_vss_per_edge_in_iteration.at(i)) + 1;
        }
      }
      obj_lb = std::max(obj_lb_tmp, obj_lb);

      if (obj_lb + GRB_EPS >= obj_ub && (sol_object->has_solution())) {
        PLOGD << "Break because obj_lb (" << obj_lb << ") >= obj_ub (" << obj_ub
              << ") -> Proven optimal";
        sol_object->set_status(SolutionStatus::Optimal);
        break;
      }

      if (optimality_strategy != OptimalityStrategy::Optimal &&
          (m_model->get(GRB_IntAttr_SolCount) >= 1)) {
        PLOGD << "Break because of feasible solution and not searching for "
                 "optimality.";
        break;
      }

      GRBLinExpr cut_expr = 0;
      for (int i = 0; i < relevant_edges.size(); ++i) {
        if (update_vss(i, obj_ub, cut_expr)) {
          reoptimize = true;
        }
      }

      if (!reoptimize) {
        PLOGD << "Break because no more VSS can be added";
        break;
      }

      m_model->addConstr(m_objective_expr, GRB_GREATER_EQUAL, obj_lb,
                         "obj_lb_" + std::to_string(obj_lb) + "_" +
                             std::to_string(iteration_number));
      m_model->addConstr(m_objective_expr, GRB_LESS_EQUAL, obj_ub,
                         "obj_ub_" + std::to_string(obj_ub) + "_" +
                             std::to_string(iteration_number));
      PLOGD << "Added constraint: obj >= " << obj_lb;
      PLOGD << "Added constraint: obj <= " << obj_ub;

      if (this->iterative_include_cuts_tmp) {
        iterative_cuts.push_back(
            m_model->addConstr(cut_expr, GRB_GREATER_EQUAL, 1,
                               "cut_" + std::to_string(iteration_number)));
        m_model->reset(1);
        PLOGD << "Added constraint: cut_expr >= 1";
      } else {
        PLOGD << "Remove " << iterative_cuts.size() << " cut constraints";
        for (const auto& c : iterative_cuts) {
          m_model->remove(c);
        }
        iterative_cuts.clear();
      }

      if (time_limit > 0) {
        const auto current_time = std::chrono::high_resolution_clock::now();
        const auto current_time_span =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time -
                                                                  m_start)
                .count();

        auto time_left = time_limit - (current_time_span / 1000);

        if (time_left < 0) {
          PLOGD << "Break because of timeout";
          if (sol_object->has_solution()) {
            PLOGD << "However, use previous obtained solution";
            break;
          }
          sol_object->set_status(SolutionStatus::Timeout);
          break;
        }

        m_model->set(GRB_DoubleParam_TimeLimit, static_cast<double>(time_left));

        PLOGD << "Next iterations limit: " << time_left << " s";
      }

      m_model->update();
    }
  }

  // Extract solving times
  IF_PLOG(plog::debug) {
    m_model_solved = std::chrono::high_resolution_clock::now();
    m_solve_time   = std::chrono::duration_cast<std::chrono::milliseconds>(
                         m_model_solved - m_model_created)
                         .count();
    PLOGD << "Model created in "
          << (static_cast<double>(m_create_time) / 1000.0) << " s";
    PLOGD << "Model solved in " << (static_cast<double>(m_solve_time) / 1000.0)
          << " s";
    PLOGD << "Total time "
          << (static_cast<double>(m_create_time + m_solve_time) / 1000.0)
          << " s";
  }

  return sol_object;
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    export_lp_if_applicable(const SolutionSettings& solution_settings) {
  if (export_option == ExportOption::ExportLP ||
      export_option == ExportOption::ExportSolutionAndLP ||
      export_option == ExportOption::ExportSolutionWithInstanceAndLP) {
    PLOGI << "Saving m_model and solution";
    const std::filesystem::path path = solution_settings.path;

    if (!is_directory_and_create(path)) {
      PLOGE << "Could not create directory " << path.string();
      throw exceptions::ExportException("Could not create directory " +
                                        path.string());
    }

    m_model->write((path / (solution_settings.name + ".mps")).string());
    if (m_model->get(GRB_IntAttr_SolCount) >= 1) {
      m_model->write((path / (solution_settings.name + ".sol")).string());
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    export_solution_if_applicable(
        const std::optional<
            cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance>&
                                sol_object,
        const SolutionSettings& solution_settings) {
  if (export_option == ExportOption::ExportSolution ||
      export_option == ExportOption::ExportSolutionWithInstance ||
      export_option == ExportOption::ExportSolutionAndLP ||
      export_option == ExportOption::ExportSolutionWithInstanceAndLP) {
    const bool export_instance =
        (export_option == ExportOption::ExportSolutionWithInstance ||
         export_option == ExportOption::ExportSolutionWithInstanceAndLP);
    PLOGI << "Saving solution";
    std::filesystem::path path = solution_settings.path;
    path /= solution_settings.name;
    sol_object->export_solution(path.parent_path(), path.filename().string(),
                                export_instance, {});
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-unchecked-optional-access)
