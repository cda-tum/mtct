#include "CustomExceptions.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <cmath>
#include <unordered_map>

std::vector<size_t>
cda_rail::solver::mip_based::VSSGenTimetableSolver::unbreakable_section_indices(
    size_t train_index) const {
  /**
   * This function returns the indices of the unbreakable sections that are
   * traversed by the train with index train_index
   * @param train_index index of the train
   * @return vector of indices
   */

  std::vector<size_t> indices;
  const auto& tr_name  = instance.get_train_list().get_train(train_index).name;
  const auto& tr_route = instance.get_route(tr_name).get_edges();
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

  const auto& train_list  = instance.get_train_list();
  const auto  tr_name     = train_list.get_train(tr).name;
  const auto& tr_schedule = instance.get_schedule(tr_name);

  s.to_use   = true;
  s.t_before = train_interval[tr].first;
  s.t_after  = train_interval[tr].second + 1;
  s.v_before = tr_schedule.v_0;
  s.v_after  = tr_schedule.v_n;

  for (const auto& tr_stop : tr_schedule.stops) {
    const auto t0 = tr_stop.begin / dt;
    const auto t1 =
        static_cast<int>(std::ceil(static_cast<double>(tr_stop.end) / dt));
    if (t >= t0 && t <= t1) {
      s.to_use = false;
      return s;
    }
    if (t0 < t && t0 > s.t_before) {
      s.t_before = t0;
      s.edges_before =
          instance.get_station_list().get_station(tr_stop.station).tracks;
      s.v_before = 0;
    }
    if (t1 > t && t1 < s.t_after) {
      s.t_after = t1;
      s.edges_after =
          instance.get_station_list().get_station(tr_stop.station).tracks;
      s.v_after = 0;
    }
  }

  return s;
}

double
cda_rail::solver::mip_based::VSSGenTimetableSolver::max_distance_travelled(
    const size_t& tr, const size_t& time_steps, const double& v0,
    const double& a_max, const bool& braking_distance) const {
  const auto& train_object = instance.get_train_list().get_train(tr);
  const auto& v_max        = train_object.max_speed;
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
    ret_val += final_speed * final_speed / (2 * train_object.deceleration);
  }
  return ret_val;
}

std::pair<std::vector<std::vector<size_t>>, std::vector<std::vector<size_t>>>
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

  std::pair<std::vector<std::vector<size_t>>, std::vector<std::vector<size_t>>>
                                                  ret_val;
  std::unordered_map<size_t, std::vector<size_t>> entry_vertices;
  std::unordered_map<size_t, std::vector<size_t>> exit_vertices;

  for (size_t tr = 0; tr < num_tr; ++tr) {
    entry_vertices[instance.get_schedule(tr).entry].push_back(tr);
    exit_vertices[instance.get_schedule(tr).exit].push_back(tr);
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

void cda_rail::solver::mip_based::VSSGenTimetableSolver::cleanup(
    const std::optional<instances::VSSGenerationTimetable>& old_instance) {
  if (old_instance.has_value()) {
    instance = old_instance.value();
  }
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
  fix_routes             = false;
  vss_model              = vss::Model(vss::ModelType::Continuous);
  include_train_dynamics = false;
  use_pwl                = false;
  use_schedule_cuts      = false;
  breakable_edge_indices.clear();
  fwd_bwd_sections.clear();
  objective_expr = 0;
  vars.clear();
  model.reset();
  env.reset();
}

bool cda_rail::solver::mip_based::VSSGenTimetableSolver::update_vss(
    size_t relevant_edge_index, double obj_ub, GRBLinExpr& cut_expr) {
  const auto& e            = relevant_edges.at(relevant_edge_index);
  const auto  vss_number_e = instance.n().max_vss_on_edge(e);
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

  auto target_vss_number_e = (model->get(GRB_IntAttr_SolCount) >= 1)
                                 ? static_cast<size_t>(std::round(obj_ub - 1))
                                 : current_vss_number_e + increase_val;

  if (target_vss_number_e >= vss_number_e) {
    target_vss_number_e = vss_number_e;
  }
  if (target_vss_number_e <= current_vss_number_e) {
    return false;
  }

  update_max_vss_on_edge(relevant_edge_index, target_vss_number_e, cut_expr);
  return true;
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::update_max_vss_on_edge(
    size_t relevant_edge_index, size_t new_max_vss, GRBLinExpr& cut_expr) {
  const auto& e            = relevant_edges.at(relevant_edge_index);
  const auto  vss_number_e = instance.n().max_vss_on_edge(e);
  const auto  old_max_vss =
      max_vss_per_edge_in_iteration.at(relevant_edge_index);
  max_vss_per_edge_in_iteration[relevant_edge_index] = new_max_vss;

  if (debug) {
    const auto& u =
        instance.n().get_vertex(instance.n().get_edge(e).source).name;
    const auto& v =
        instance.n().get_vertex(instance.n().get_edge(e).target).name;
    std::cout << "Update possible VSS on edge " << u << " -> " << v << " from "
              << old_max_vss << " to " << new_max_vss << std::endl;
  }

  if (this->vss_model.get_model_type() == vss::ModelType::Inferred) {
    vars.at("num_vss_segments")(relevant_edge_index)
        .set(GRB_DoubleAttr_UB, static_cast<double>(new_max_vss) + 1);
    if (this->iterative_include_cuts && new_max_vss > old_max_vss) {
      const auto b =
          model->addVar(0, 1, 0, GRB_BINARY,
                        "binary_cut_" + std::to_string(relevant_edge_index) +
                            "_" + std::to_string(old_max_vss));
      // b = 1 iff num_vss_segments(relevant_edge_index) >= old_max_vss + 1
      model->addConstr(vars.at("num_vss_segments")(relevant_edge_index) -
                               static_cast<double>(old_max_vss) <=
                           (vss_number_e + 1) * b,
                       "binary_cut_relation_" +
                           std::to_string(relevant_edge_index) + "_" +
                           std::to_string(old_max_vss) + "_1");
      model->addConstr(
          static_cast<double>(old_max_vss + 1) -
                  vars.at("num_vss_segments")(relevant_edge_index) <=
              (vss_number_e) * (1 - b),
          "binary_cut_relation_" + std::to_string(relevant_edge_index) + "_" +
              std::to_string(old_max_vss) + "_2");
      cut_expr += b;
      if (debug) {
        std::cout << "Add binary_cut_" << relevant_edge_index << "_"
                  << old_max_vss << "to cut_expr" << std::endl;
      }
    }
  }
  if (this->vss_model.get_model_type() == vss::ModelType::Continuous) {
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      vars.at("b_used")(relevant_edge_index, vss)
          .set(GRB_DoubleAttr_UB, static_cast<double>(vss < new_max_vss));
    }
    if (this->iterative_include_cuts && new_max_vss > old_max_vss) {
      cut_expr += vars.at("b_used")(relevant_edge_index, old_max_vss);
      if (debug) {
        std::cout << "Add b_used(" << relevant_edge_index << "," << old_max_vss
                  << ") to cut_expr" << std::endl;
      }
    }
  }
  if (this->vss_model.get_model_type() == vss::ModelType::InferredAlt) {
    for (size_t sep_type = 0;
         sep_type < this->vss_model.get_separation_functions().size();
         ++sep_type) {
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        vars.at("type_num_vss_segments")(relevant_edge_index, sep_type, vss)
            .set(GRB_DoubleAttr_UB, static_cast<double>(vss < new_max_vss));
      }
      if (this->iterative_include_cuts && new_max_vss > old_max_vss) {
        cut_expr += vars.at("type_num_vss_segments")(relevant_edge_index,
                                                     sep_type, old_max_vss);
        if (debug) {
          std::cout << "Add type_num_vss_segments(" << relevant_edge_index
                    << "," << sep_type << "," << old_max_vss << ") to cut_expr"
                    << std::endl;
        }
      }
    }
  }
}
