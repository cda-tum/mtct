#include "CustomExceptions.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <cmath>
#include <plog/Log.h>
#include <unordered_map>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

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
  s.v_before = tr_schedule.get_v_0();
  s.v_after  = tr_schedule.get_v_n();

  for (const auto& tr_stop : tr_schedule.get_stops()) {
    const auto t0 = tr_stop.arrival() / dt;
    const auto t1 = static_cast<int>(
        std::ceil(static_cast<double>(tr_stop.departure()) / dt));
    if (t >= t0 && t <= t1) {
      s.to_use = false;
      return s;
    }
    if (t0 < t && t0 > s.t_before) {
      s.t_before     = t0;
      s.edges_before = instance.get_station_list()
                           .get_station(tr_stop.get_station_name())
                           .tracks;
      s.v_before = 0;
    }
    if (t1 > t && t1 < s.t_after) {
      s.t_after     = t1;
      s.edges_after = instance.get_station_list()
                          .get_station(tr_stop.get_station_name())
                          .tracks;
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
    entry_vertices[instance.get_schedule(tr).get_entry()].push_back(tr);
    exit_vertices[instance.get_schedule(tr).get_exit()].push_back(tr);
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
  objective_expr = 0;
  model->reset(1);
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

  IF_PLOG(plog::debug) {
    const auto& u =
        instance.n().get_vertex(instance.n().get_edge(e).source).name;
    const auto& v =
        instance.n().get_vertex(instance.n().get_edge(e).target).name;
    PLOGD << "Update possible VSS on edge " << u << " -> " << v << " from "
          << old_max_vss << " to " << new_max_vss;
  }

  if (this->vss_model.get_model_type() == vss::ModelType::Inferred) {
    vars.at("num_vss_segments")(relevant_edge_index)
        .set(GRB_DoubleAttr_UB, static_cast<double>(new_max_vss) + 1);
    if (this->iterative_include_cuts_tmp && new_max_vss > old_max_vss) {
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
      PLOGD << "Add binary_cut_" << relevant_edge_index << "_" << old_max_vss
            << "to cut_expr";
    }
  }
  if (this->vss_model.get_model_type() == vss::ModelType::Continuous) {
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      vars.at("b_used")(relevant_edge_index, vss)
          .set(GRB_DoubleAttr_UB, static_cast<double>(vss < new_max_vss));
    }
    if (this->iterative_include_cuts_tmp && new_max_vss > old_max_vss) {
      cut_expr += vars.at("b_used")(relevant_edge_index, old_max_vss);
      PLOGD << "Add b_used(" << relevant_edge_index << "," << old_max_vss
            << ") to cut_expr";
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
      if (this->iterative_include_cuts_tmp && new_max_vss > old_max_vss) {
        cut_expr += vars.at("type_num_vss_segments")(relevant_edge_index,
                                                     sep_type, old_max_vss);
        PLOGD << "Add type_num_vss_segments(" << relevant_edge_index << ","
              << sep_type << "," << old_max_vss << ") to cut_expr";
      }
    }
  }
}

std::optional<cda_rail::instances::VSSGenerationTimetable>
cda_rail::solver::mip_based::VSSGenTimetableSolver::initialize_variables(
    const cda_rail::solver::mip_based::ModelDetail&      model_detail,
    const cda_rail::solver::mip_based::ModelSettings&    model_settings,
    const cda_rail::solver::mip_based::SolverStrategy&   solver_strategy,
    const cda_rail::solver::mip_based::SolutionSettings& solution_settings,
    int time_limit, bool debug_input) {
  this->solve_init_vss_gen_timetable(time_limit, debug_input);

  if (!model_settings.model_type.check_consistency()) {
    PLOGE << "Model type  and separation types/functions are not consistent.";
    throw cda_rail::exceptions::ConsistencyException(
        "Model type and separation types/functions are not consistent.");
  }

  if (!instance.n().is_consistent_for_transformation()) {
    PLOGE << "Instance is not consistent for transformation.";
    throw exceptions::ConsistencyException();
  }

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

  if (this->fix_routes && !instance.has_route_for_every_train()) {
    PLOGE << "Instance does not have a route for every train";
    throw exceptions::ConsistencyException(
        "Instance does not have a route for every train");
  }

  std::optional<instances::VSSGenerationTimetable> old_instance;
  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    PLOGI << "Preprocessing graph...";
    old_instance = instance;
    instance.discretize(this->vss_model.get_separation_functions().front());
    PLOGI << "Preprocessing graph... DONE";
  }

  PLOGI << "Creating model...";
  PLOGD << "Initialize other relevant variables";

  num_t = static_cast<size_t>(instance.max_t() / dt);
  if (instance.max_t() % dt != 0) {
    num_t += 1;
  }

  num_tr       = instance.get_train_list().size();
  num_edges    = instance.n().number_of_edges();
  num_vertices = instance.n().number_of_vertices();

  unbreakable_sections = instance.n().unbreakable_sections();

  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    no_border_vss_sections = instance.n().no_border_vss_sections();
    num_breakable_sections = no_border_vss_sections.size();
    no_border_vss_vertices =
        instance.n().get_vertices_by_type(VertexType::NoBorderVSS);
  } else {
    breakable_edges = instance.n().breakable_edges();
    for (size_t i = 0; i < breakable_edges.size(); ++i) {
      breakable_edge_indices[breakable_edges[i]] = i;
    }
    breakable_edges_pairs = instance.n().combine_reverse_edges(breakable_edges);
    num_breakable_sections = breakable_edges.size();
    relevant_edges         = instance.n().relevant_breakable_edges();
  }

  for (size_t i = 0; i < num_tr; ++i) {
    train_interval.emplace_back(instance.time_index_interval(i, dt, false));
  }

  if (iterative_vss && vss_model.get_model_type() == vss::ModelType::Discrete) {
    PLOGE << "Iterative VSS not supported for discrete VSS model";
    throw exceptions::ConsistencyException(
        "Iterative VSS not supported for discrete VSS model");
  }

  max_vss_per_edge_in_iteration.resize(relevant_edges.size(), 0);
  for (size_t i = 0; i < relevant_edges.size(); ++i) {
    const auto& e            = relevant_edges.at(i);
    const auto  vss_number_e = instance.n().max_vss_on_edge(e);
    if (iterative_vss) {
      if (iterative_update_strategy == UpdateStrategy::Fixed) {
        max_vss_per_edge_in_iteration[i] = std::min(
            vss_number_e, static_cast<int>(std::ceil(iterative_initial_value)));
      } else if (iterative_update_strategy == UpdateStrategy::Relative) {
        max_vss_per_edge_in_iteration[i] = std::min(
            vss_number_e,
            static_cast<int>(std::ceil(iterative_initial_value *
                                       static_cast<double>(vss_number_e))));
      } else {
        PLOGE << "Unknown update strategy";
        throw exceptions::ConsistencyException("Unknown update strategy");
      }
    } else {
      max_vss_per_edge_in_iteration[i] = vss_number_e;
    }
  }

  calculate_fwd_bwd_sections();

  return old_instance;
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::set_timeout(
    int time_limit) {
  PLOGI << "DONE creating model";

  if (plog::get()->checkSeverity(plog::debug) || time_limit > 0) {
    model_created = std::chrono::high_resolution_clock::now();
    create_time   = std::chrono::duration_cast<std::chrono::milliseconds>(
                      model_created - start)
                      .count();

    auto time_left = time_limit - create_time / 1000;
    if (time_left < 0 && time_limit > 0) {
      time_left = 1;
    }
    if (time_limit > 0) {
      model->set(GRB_DoubleParam_TimeLimit, static_cast<double>(time_left));
    }
    PLOGD << "Model created in " << (static_cast<double>(create_time) / 1000.0)
          << " s";
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
    model->set(GRB_IntParam_NonConvex, 2);
  }
}

std::optional<cda_rail::instances::SolVSSGenerationTimetable>
cda_rail::solver::mip_based::VSSGenTimetableSolver::optimize(
    const std::optional<instances::VSSGenerationTimetable>& old_instance,
    int                                                     time_limit) {
  std::optional<instances::SolVSSGenerationTimetable> sol_object;

  bool reoptimize = true;

  double obj_ub = 1.0;
  for (const auto& e : relevant_edges) {
    obj_ub += instance.const_n().max_vss_on_edge(e);
  }
  double obj_lb           = 0;
  size_t iteration_number = 0;

  std::vector<GRBConstr> iterative_cuts;
  this->iterative_include_cuts_tmp = this->iterative_include_cuts;

  while (reoptimize) {
    reoptimize = false;

    if (optimality_strategy == OptimalityStrategy::Feasible) {
      model->set(GRB_IntParam_SolutionLimit, 1);
      model->set(GRB_IntParam_MIPFocus, 1);
      PLOGD << "Settings focussing on feasibility";
    }

    this->model->optimize();
    iteration_number += 1;

    if (model->get(GRB_IntAttr_SolCount) >= 1) {
      const auto obj_tmp = model->get(GRB_DoubleAttr_ObjVal);
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
      if (model->get(GRB_IntAttr_Status) == GRB_TIME_LIMIT) {
        PLOGD << "Break because of timeout";
        if (sol_object->has_solution()) {
          PLOGD << "However, use previous obtained solution";
          break;
        }
        sol_object =
            extract_solution(postprocess, !iterative_vss, old_instance);
        break;
      }

      auto obj_lb_tmp = model->get(GRB_DoubleAttr_ObjBound);
      for (int i = 0; i < relevant_edges.size(); ++i) {
        if (static_cast<double>(max_vss_per_edge_in_iteration.at(i)) + 1 <
                obj_lb_tmp &&
            max_vss_per_edge_in_iteration.at(i) <
                instance.const_n().max_vss_on_edge(relevant_edges.at(i))) {
          obj_lb_tmp =
              static_cast<double>(max_vss_per_edge_in_iteration.at(i)) + 1;
        }
      }
      if (obj_lb_tmp > obj_lb) {
        obj_lb = obj_lb_tmp;
      }

      if (obj_lb + GRB_EPS >= obj_ub && (sol_object->has_solution())) {
        PLOGD << "Break because obj_lb (" << obj_lb << ") >= obj_ub (" << obj_ub
              << ") -> Proven optimal";
        sol_object->set_status(SolutionStatus::Optimal);
        break;
      }

      if (optimality_strategy != OptimalityStrategy::Optimal &&
          (model->get(GRB_IntAttr_SolCount) >= 1)) {
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

      model->addConstr(objective_expr, GRB_GREATER_EQUAL, obj_lb,
                       "obj_lb_" + std::to_string(obj_lb) + "_" +
                           std::to_string(iteration_number));
      model->addConstr(objective_expr, GRB_LESS_EQUAL, obj_ub,
                       "obj_ub_" + std::to_string(obj_ub) + "_" +
                           std::to_string(iteration_number));
      PLOGD << "Added constraint: obj >= " << obj_lb;
      PLOGD << "Added constraint: obj <= " << obj_ub;

      if (this->iterative_include_cuts_tmp) {
        iterative_cuts.push_back(
            model->addConstr(cut_expr, GRB_GREATER_EQUAL, 1,
                             "cut_" + std::to_string(iteration_number)));
        model->reset(1);
        PLOGD << "Added constraint: cut_expr >= 1";
      } else {
        PLOGD << "Remove " << iterative_cuts.size() << " cut constraints";
        for (const auto& c : iterative_cuts) {
          model->remove(c);
        }
        iterative_cuts.clear();
      }

      if (time_limit > 0) {
        const auto current_time = std::chrono::high_resolution_clock::now();
        const auto current_time_span =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time -
                                                                  start)
                .count();

        auto time_left = time_limit - current_time_span / 1000;

        if (time_left < 0) {
          PLOGD << "Break because of timeout";
          if (sol_object->has_solution()) {
            PLOGD << "However, use previous obtained solution";
            break;
          }
          sol_object->set_status(SolutionStatus::Timeout);
          break;
        }

        model->set(GRB_DoubleParam_TimeLimit, static_cast<double>(time_left));

        PLOGD << "Next iterations limit: " << time_left << " s";
      }

      model->update();
    }
  }

  IF_PLOG(plog::debug) {
    model_solved = std::chrono::high_resolution_clock::now();
    solve_time   = std::chrono::duration_cast<std::chrono::milliseconds>(
                     model_solved - model_created)
                     .count();
    PLOGD << "Model created in " << (static_cast<double>(create_time) / 1000.0)
          << " s";
    PLOGD << "Model solved in " << (static_cast<double>(solve_time) / 1000.0)
          << " s";
    PLOGD << "Total time "
          << (static_cast<double>(create_time + solve_time) / 1000.0) << " s";
  }

  return sol_object;
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    export_lp_if_applicable(const SolutionSettings& solution_settings) {
  if (export_option == ExportOption::ExportLP ||
      export_option == ExportOption::ExportSolutionAndLP ||
      export_option == ExportOption::ExportSolutionWithInstanceAndLP) {
    PLOGI << "Saving model and solution";
    std::filesystem::path path = solution_settings.path;

    if (!is_directory_and_create(path)) {
      PLOGE << "Could not create directory " << path.string();
      throw exceptions::ExportException("Could not create directory " +
                                        path.string());
    }

    model->write((path / (solution_settings.name + ".mps")).string());
    if (model->get(GRB_IntAttr_SolCount) >= 1) {
      model->write((path / (solution_settings.name + ".sol")).string());
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    export_solution_if_applicable(
        const std::optional<cda_rail::instances::SolVSSGenerationTimetable>&
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
    sol_object->export_solution(path, export_instance);
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
