#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "nlohmann/json.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <cmath>
#include <fstream>
#include <plog/Log.h>
#include <unordered_set>
#include <vector>

using json = nlohmann::json;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

// NOLINTBEGIN(performance-inefficient-string-concatenation)

cda_rail::instances::SolVSSGenerationTimetable::SolVSSGenerationTimetable(
    cda_rail::instances::VSSGenerationTimetable instance, int dt)
    : SolGeneralProblemInstance<VSSGenerationTimetable>(instance), dt(dt) {
  this->initialize_vectors();
}

double
cda_rail::instances::SolVSSGenerationTimetable::get_train_pos(size_t train_id,
                                                              int time) const {
  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  const auto& [t0, tn] = instance.time_index_interval(train_id, dt, true);
  if (t0 * dt > time || tn * dt < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt == 0) {
    const auto t_index = static_cast<size_t>(time / dt) - t0;
    return train_pos.at(train_id).at(t_index);
  }

  const auto t_1 = static_cast<size_t>(std::floor(time / dt)) - t0;
  const auto t_2 = t_1 + 1;

  const auto x_1 = train_pos.at(train_id).at(t_1);
  const auto v_1 = train_speed.at(train_id).at(t_1);
  const auto x_2 = train_pos.at(train_id).at(t_2);
  const auto v_2 = train_speed.at(train_id).at(t_2);

  if (approx_equal(x_2 - x_1, 0.5 * dt * (v_1 + v_2))) {
    const auto a   = (v_2 - v_1) / dt;
    const auto tau = time - static_cast<double>(t0 + t_1) * dt;
    return x_1 + v_1 * tau + 0.5 * a * tau * tau;
  }

  throw exceptions::ConsistencyException(
      "Train " + std::to_string(train_id) + " is not scheduled at time " +
      std::to_string(time) + " and cannot be inferred by linear interpolation");
}

double cda_rail::instances::SolVSSGenerationTimetable::get_train_speed(
    size_t train_id, int time) const {
  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  const auto& [t0, tn] = instance.time_index_interval(train_id, dt, true);
  if (t0 * dt > time || tn * dt < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt == 0) {
    const auto t_index = static_cast<size_t>(time / dt) - t0;
    return train_speed.at(train_id).at(t_index);
  }

  const auto t_1 = static_cast<size_t>(std::floor(time / dt)) - t0;
  const auto t_2 = t_1 + 1;

  const auto x_1 = train_pos.at(train_id).at(t_1);
  const auto v_1 = train_speed.at(train_id).at(t_1);
  const auto x_2 = train_pos.at(train_id).at(t_2);
  const auto v_2 = train_speed.at(train_id).at(t_2);

  if (approx_equal(x_2 - x_1, 0.5 * dt * (v_1 + v_2))) {
    const auto a   = (v_2 - v_1) / dt;
    const auto tau = time - static_cast<double>(t0 + t_1) * dt;
    return v_1 + a * tau;
  }

  throw exceptions::ConsistencyException(
      "Train " + std::to_string(train_id) + " is not scheduled at time " +
      std::to_string(time) + " and cannot be inferred by linear interpolation");
}

void cda_rail::instances::SolVSSGenerationTimetable::add_vss_pos(
    size_t edge_id, double pos, bool reverse_edge) {
  if (!instance.const_n().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  const auto& edge = instance.const_n().get_edge(edge_id);

  if (pos <= EPS || pos + EPS >= edge.length) {
    throw exceptions::ConsistencyException(
        "VSS position " + std::to_string(pos) + " is not on edge " +
        std::to_string(edge_id));
  }

  vss_pos.at(edge_id).emplace_back(pos);
  std::sort(vss_pos.at(edge_id).begin(), vss_pos.at(edge_id).end());

  if (reverse_edge) {
    const auto reverse_edge_index =
        instance.const_n().get_reverse_edge_index(edge_id);
    if (reverse_edge_index.has_value()) {
      vss_pos.at(reverse_edge_index.value()).emplace_back(edge.length - pos);
      std::sort(vss_pos.at(reverse_edge_index.value()).begin(),
                vss_pos.at(reverse_edge_index.value()).end());
    }
  }
}

void cda_rail::instances::SolVSSGenerationTimetable::set_vss_pos(
    size_t edge_id, std::vector<double> pos) {
  if (!instance.const_n().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  const auto& edge = instance.const_n().get_edge(edge_id);

  for (const auto& p : pos) {
    if (p <= EPS || p + EPS >= edge.length) {
      throw exceptions::ConsistencyException(
          "VSS position " + std::to_string(p) + " is not on edge " +
          std::to_string(edge_id));
    }
  }

  vss_pos.at(edge_id) = std::move(pos);
}

void cda_rail::instances::SolVSSGenerationTimetable::reset_vss_pos(
    size_t edge_id) {
  if (!instance.const_n().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  vss_pos.at(edge_id).clear();
}

void cda_rail::instances::SolVSSGenerationTimetable::add_train_pos(
    size_t train_id, int time, double pos) {
  if (pos + EPS < 0) {
    throw exceptions::ConsistencyException(
        "Train position " + std::to_string(pos) + " is negative");
  }

  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  const auto& [t0, tn] = instance.time_index_interval(train_id, dt, true);
  if (t0 * dt > time || tn * dt < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt != 0) {
    throw exceptions::ConsistencyException(
        "Time " + std::to_string(time) +
        " is not a multiple of dt = " + std::to_string(dt));
  }

  const auto tr_t0   = instance.time_index_interval(train_id, dt, true).first;
  const auto t_index = static_cast<size_t>(time / dt) - tr_t0;
  train_pos.at(train_id).at(t_index) = pos;
}

void cda_rail::instances::SolVSSGenerationTimetable::add_train_speed(
    size_t train_id, int time, double speed) {
  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  if (speed + EPS < 0) {
    throw exceptions::ConsistencyException(
        "Train speed " + std::to_string(speed) + " is negative");
  }
  if (speed > instance.get_train_list().get_train(train_id).max_speed + EPS) {
    throw exceptions::ConsistencyException(
        "Train speed " + std::to_string(speed) +
        " is greater than the maximum speed of train " +
        std::to_string(train_id) + " (" +
        std::to_string(
            instance.get_train_list().get_train(train_id).max_speed) +
        ")");
  }

  const auto& [t0, tn] = instance.time_index_interval(train_id, dt, true);
  if (t0 * dt > time || tn * dt < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt != 0) {
    throw exceptions::ConsistencyException(
        "Time " + std::to_string(time) +
        " is not a multiple of dt = " + std::to_string(dt));
  }

  const auto tr_t0   = instance.time_index_interval(train_id, dt, true).first;
  const auto t_index = static_cast<size_t>(time / dt) - tr_t0;
  train_speed.at(train_id).at(t_index) = speed;
}

bool cda_rail::instances::SolVSSGenerationTimetable::check_consistency() const {
  if (status == SolutionStatus::Unknown) {
    return false;
  }
  if (status == SolutionStatus::Infeasible ||
      status == SolutionStatus::Timeout) {
    return true;
  }
  if (obj + EPS < 0) {
    return false;
  }
  if (dt < 0) {
    return false;
  }
  if (!instance.check_consistency(true)) {
    return false;
  }
  for (const auto& train_pos_vec : train_pos) {
    for (const auto& pos : train_pos_vec) {
      if (pos + EPS < 0) {
        return false;
      }
    }
  }
  for (size_t tr_id = 0; tr_id < train_speed.size(); ++tr_id) {
    const auto& train = instance.get_train_list().get_train(tr_id);
    for (double v : train_speed.at(tr_id)) {
      if (v + EPS < 0 || v > train.max_speed + EPS) {
        return false;
      }
    }
  }
  for (size_t edge_id = 0; edge_id < vss_pos.size(); ++edge_id) {
    const auto& edge = instance.const_n().get_edge(edge_id);
    for (const auto& pos : vss_pos.at(edge_id)) {
      if (pos + EPS < 0 || pos > edge.length + EPS) {
        return false;
      }
    }
  }
  return true;
}

void cda_rail::instances::SolVSSGenerationTimetable::export_solution(
    const std::filesystem::path& p, bool export_instance) const {
  /**
   * This method exports the solution object to a specific path. This includes
   * the following:
   * - If export_instance is true, the instance is exported to the path p /
   * instance
   * - If export_instance is false, the routes are exported to the path p /
   * instance / routes
   * - dt, status, obj, and postprocessed are exported to p / solution /
   * data.json
   * - vss_pos is exported to p / solution / vss_pos.json
   * - train_pos and train_speed are exported to p / solution / train_pos.json
   * and p / solution / train_speed.json The method throws a
   * ConsistencyException if the solution is not consistent.
   *
   * @param p the path to the folder where the solution should be exported
   * @param export_instance whether the instance should be exported next to the
   * solution
   */

  if (!check_consistency()) {
    throw exceptions::ConsistencyException();
  }

  if (!is_directory_and_create(p / "solution")) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  if (export_instance) {
    instance.export_instance(p / "instance");
  } else {
    instance.routes.export_routes(p / "instance" / "routes",
                                  instance.const_n());
  }

  json data;
  data["dt"]            = dt;
  data["status"]        = static_cast<int>(status);
  data["obj"]           = obj;
  data["mip_obj"]       = mip_obj;
  data["postprocessed"] = postprocessed;
  data["has_solution"]  = has_sol;
  std::ofstream data_file(p / "solution" / "data.json");
  data_file << data << std::endl;
  data_file.close();

  json vss_pos_json;
  for (size_t edge_id = 0; edge_id < instance.const_n().number_of_edges();
       ++edge_id) {
    const auto& edge = instance.const_n().get_edge(edge_id);
    const auto& v0   = instance.const_n().get_vertex(edge.source).name;
    const auto& v1   = instance.const_n().get_vertex(edge.target).name;
    vss_pos_json["('" + v0 + "', '" + v1 + "')"] = vss_pos.at(edge_id);
  }

  std::ofstream vss_pos_file(p / "solution" / "vss_pos.json");
  vss_pos_file << vss_pos_json << std::endl;
  vss_pos_file.close();

  json train_pos_json;
  json train_speed_json;
  for (size_t tr_id = 0; tr_id < instance.get_train_list().size(); ++tr_id) {
    const auto& train       = instance.get_train_list().get_train(tr_id);
    const auto  tr_interval = instance.time_index_interval(tr_id, dt, true);
    json        train_pos_json_tmp;
    json        train_speed_json_tmp;
    for (size_t t_id = 0; t_id < train_pos.at(tr_id).size(); ++t_id) {
      const auto t = static_cast<int>(tr_interval.first + t_id) * dt;
      train_pos_json_tmp[std::to_string(t)]   = train_pos.at(tr_id).at(t_id);
      train_speed_json_tmp[std::to_string(t)] = train_speed.at(tr_id).at(t_id);
    }
    train_pos_json[train.name]   = train_pos_json_tmp;
    train_speed_json[train.name] = train_speed_json_tmp;
  }

  std::ofstream train_pos_file(p / "solution" / "train_pos.json");
  train_pos_file << train_pos_json << std::endl;
  train_pos_file.close();

  std::ofstream train_speed_file(p / "solution" / "train_speed.json");
  train_speed_file << train_speed_json << std::endl;
  train_speed_file.close();
}

cda_rail::instances::SolVSSGenerationTimetable::SolVSSGenerationTimetable(
    const std::filesystem::path&                 p,
    const std::optional<VSSGenerationTimetable>& instance) {
  if (!std::filesystem::exists(p)) {
    throw exceptions::ImportException("Path does not exist");
  }
  if (!std::filesystem::is_directory(p)) {
    throw exceptions::ImportException("Path is not a directory");
  }

  bool const import_routes = instance.has_value();
  if (instance.has_value()) {
    this->instance = instance.value();
  } else {
    this->instance = VSSGenerationTimetable(p / "instance");
  }

  if (import_routes) {
    this->instance.routes =
        RouteMap(p / "instance" / "routes", this->instance.const_n());
  }

  if (!this->instance.check_consistency(true)) {
    throw exceptions::ConsistencyException(
        "Imported instance is not consistent");
  }

  // Read data
  std::ifstream data_file(p / "solution" / "data.json");
  json          data  = json::parse(data_file);
  this->dt            = data["dt"].get<int>();
  this->status        = static_cast<SolutionStatus>(data["status"].get<int>());
  this->obj           = data["obj"].get<double>();
  this->mip_obj       = data["mip_obj"].get<double>();
  this->postprocessed = data["postprocessed"].get<bool>();
  this->has_sol       = data["has_solution"].get<bool>();

  this->initialize_vectors();

  // Read vss_pos
  std::ifstream vss_pos_file(p / "solution" / "vss_pos.json");
  json          vss_pos_json = json::parse(vss_pos_file);
  for (const auto& [key, val] : vss_pos_json.items()) {
    std::string source_name;
    std::string target_name;
    extract_vertices_from_key(key, source_name, target_name);
    const auto vss_pos_vector = val.get<std::vector<double>>();
    set_vss_pos(source_name, target_name, vss_pos_vector);
  }

  // Read train_pos
  std::ifstream train_pos_file(p / "solution" / "train_pos.json");
  json          train_pos_json = json::parse(train_pos_file);
  for (const auto& [tr_name, tr_pos_json] : train_pos_json.items()) {
    for (const auto& [t, pos] : tr_pos_json.items()) {
      this->add_train_pos(tr_name, std::stoi(t), pos.get<double>());
    }
  }

  // Read train_speed
  std::ifstream train_speed_file(p / "solution" / "train_speed.json");
  json          train_speed_json = json::parse(train_speed_file);
  for (const auto& [tr_name, tr_speed_json] : train_speed_json.items()) {
    for (const auto& [t, speed] : tr_speed_json.items()) {
      this->add_train_speed(tr_name, std::stoi(t), speed.get<double>());
    }
  }

  if (!this->check_consistency()) {
    throw exceptions::ConsistencyException(
        "Imported solution object is not consistent");
  }
}

void cda_rail::instances::SolVSSGenerationTimetable::initialize_vectors() {
  vss_pos = std::vector<std::vector<double>>(
      this->instance.const_n().number_of_edges());
  train_pos.reserve(this->instance.get_train_list().size());
  train_speed.reserve(this->instance.get_train_list().size());

  for (size_t tr = 0; tr < this->instance.get_train_list().size(); ++tr) {
    const auto tr_interval = this->instance.time_index_interval(tr, dt, true);
    const auto tr_interval_size = tr_interval.second - tr_interval.first + 1;
    train_pos.emplace_back(tr_interval_size, -1);
    train_speed.emplace_back(tr_interval_size, -1);
  }
}

std::vector<double>
cda_rail::instances::SolVSSGenerationTimetable::get_valid_border_stops(
    size_t train_id) const {
  const auto& tr_name  = instance.get_train_list().get_train(train_id).name;
  const auto& tr_route = instance.get_route(tr_name);
  const auto& tr_route_edges = tr_route.get_edges();

  std::vector<double> valid_border_stops;
  valid_border_stops.emplace_back(0);
  for (const auto& e : tr_route_edges) {
    const auto& edge            = instance.const_n().get_edge(e);
    const auto& e_target        = instance.const_n().get_vertex(edge.target);
    const auto [e_start, e_end] = tr_route.edge_pos(e, instance.const_n());

    const auto& vss_on_e = get_vss_pos(e);
    for (const auto& vss : vss_on_e) {
      if (vss > EPS && vss < edge.length - EPS) {
        valid_border_stops.emplace_back(e_start + vss);
      }
    }

    if (e_target.type != VertexType::NoBorder) {
      valid_border_stops.emplace_back(e_end);
    }
  }

  // Sort return value
  std::sort(valid_border_stops.begin(), valid_border_stops.end());

  return valid_border_stops;
}

cda_rail::instances::SolVSSGenerationTimetable
cda_rail::solver::mip_based::VSSGenTimetableSolver::extract_solution(
    bool postprocess, bool full_model,
    const std::optional<instances::VSSGenerationTimetable>& old_instance)
    const {
  PLOGD << "Extracting solution object...";

  auto sol_obj = instances::SolVSSGenerationTimetable(
      (old_instance.has_value() ? old_instance.value() : instance), dt);

  if (const auto grb_status = model->get(GRB_IntAttr_Status);
      full_model && grb_status == GRB_OPTIMAL) {
    PLOGD << "Solution status: Optimal";
    sol_obj.set_status(SolutionStatus::Optimal);
  } else if (grb_status == GRB_INFEASIBLE) {
    PLOGD << "Solution status: Infeasible";
    sol_obj.set_status(SolutionStatus::Infeasible);
  } else if (model->get(GRB_IntAttr_SolCount) >= 1) {
    PLOGD << "Solution status: Feasible (optimality unknown)";
    sol_obj.set_status(SolutionStatus::Feasible);
  } else if (grb_status == GRB_TIME_LIMIT &&
             model->get(GRB_IntAttr_SolCount) == 0) {
    PLOGD << "Solution status: Timeout (Feasibility unknown)";
    sol_obj.set_status(SolutionStatus::Timeout);
  } else {
    PLOGE << "Solution status code " << grb_status << " unknown";
    throw exceptions::ConsistencyException(
        "Gurobi status code " + std::to_string(grb_status) + " unknown.");
  }

  if (const auto sol_count = model->get(GRB_IntAttr_SolCount);
      sol_count < 0.5) {
    return sol_obj;
  }

  const auto mip_obj_val =
      static_cast<int>(std::round(model->get(GRB_DoubleAttr_ObjVal)));
  sol_obj.set_mip_obj(mip_obj_val);
  PLOGD << "MIP objective: " << mip_obj_val;

  if (vss_model.get_model_type() == vss::ModelType::Discrete) {
    // TODO: Implement
    sol_obj.set_obj(mip_obj_val);
    return sol_obj;
  }

  sol_obj.set_solution_found();

  int obj = 0;

  for (size_t r_e_index = 0; r_e_index < relevant_edges.size(); ++r_e_index) {
    const auto  e_index      = relevant_edges.at(r_e_index);
    const auto  vss_number_e = instance.const_n().max_vss_on_edge(e_index);
    const auto& e            = instance.const_n().get_edge(e_index);
    const auto  reverse_edge_index =
        instance.const_n().get_reverse_edge_index(e_index);
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      bool b_used = false;

      if (vss_model.get_model_type() == vss::ModelType::Continuous) {
        b_used =
            vars.at("b_used").at(r_e_index, vss).get(GRB_DoubleAttr_X) > 0.5;
      } else if (vss_model.get_model_type() == vss::ModelType::Inferred) {
        b_used =
            vars.at("num_vss_segments").at(r_e_index).get(GRB_DoubleAttr_X) >
            static_cast<double>(vss) + 1.5;
      } else if (vss_model.get_model_type() == vss::ModelType::InferredAlt) {
        // if any of "type_num_vss_segments"(r_e_index, sep_type_index, num_vss)
        // is > 0.5 for vss <= num_vss < vss_number_e for sep_type_index = 0,
        // ..., num_sep_types - 1 then b_used = true
        for (size_t sep_type_index = 0;
             sep_type_index < vss_model.get_separation_functions().size();
             ++sep_type_index) {
          for (size_t num_vss = vss; num_vss < vss_number_e; ++num_vss) {
            if (vars.at("type_num_vss_segments")
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
          const auto& source = instance.const_n().get_vertex(e.source).name;
          const auto& target = instance.const_n().get_vertex(e.target).name;
          PLOGD << "Postprocessing on " << source << " to " << target;
        }
        b_used = false;
        for (size_t tr = 0; tr < num_tr; ++tr) {
          for (size_t t = train_interval.at(tr).first;
               t <= train_interval.at(tr).second; ++t) {
            const auto front1 =
                vars.at("b_front")
                    .at(tr, t, breakable_edge_indices.at(e_index), vss)
                    .get(GRB_DoubleAttr_X) > 0.5;
            const auto rear1 =
                vars.at("b_rear")
                    .at(tr, t, breakable_edge_indices.at(e_index), vss)
                    .get(GRB_DoubleAttr_X) > 0.5;
            const auto front2 =
                (reverse_edge_index.has_value() &&
                 !instance
                      .trains_on_edge(reverse_edge_index.value(), fix_routes,
                                      {tr})
                      .empty())
                    ? vars.at("b_front")
                              .at(tr, t,
                                  breakable_edge_indices.at(
                                      reverse_edge_index.value()),
                                  vss)
                              .get(GRB_DoubleAttr_X) > 0.5
                    : false;
            const auto rear2 =
                (reverse_edge_index.has_value() &&
                 !instance
                      .trains_on_edge(reverse_edge_index.value(), fix_routes,
                                      {tr})
                      .empty())
                    ? vars.at("b_rear")
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

      const auto b_pos_val =
          round_to(vars.at("b_pos")
                       .at(breakable_edge_indices.at(e_index), vss)
                       .get(GRB_DoubleAttr_X),
                   ROUNDING_PRECISION);
      IF_PLOG(plog::debug) {
        const auto& source = instance.const_n().get_vertex(e.source).name;
        const auto& target = instance.const_n().get_vertex(e.target).name;
        PLOGD << "Add VSS at " << b_pos_val << " on " << source << " to "
              << target;
      }
      sol_obj.add_vss_pos(e_index, b_pos_val, true);
      obj += 1;
    }
  }

  sol_obj.set_obj(obj);
  sol_obj.set_postprocessed(postprocess);

  if (!fix_routes) {
    sol_obj.reset_routes();
    PLOGD << "Extracting routes";
    for (size_t tr = 0; tr < num_tr; ++tr) {
      const auto train = instance.get_train_list().get_train(tr);
      sol_obj.add_empty_route(train.name);
      size_t current_vertex = instance.get_schedule(tr).get_entry();
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        std::unordered_set<size_t> edge_list;
        for (int e = 0; e < num_edges; ++e) {
          const auto tr_on_edge =
              vars.at("x").at(tr, t, e).get(GRB_DoubleAttr_X) > 0.5;
          if (tr_on_edge &&
              !sol_obj.get_instance().get_route(train.name).contains_edge(e) &&
              edge_list.count(e) == 0) {
            edge_list.emplace(e);
          }
        }
        while (!edge_list.empty()) {
          bool edge_added = false;
          for (const auto& e : edge_list) {
            if (instance.const_n().get_edge(e).source == current_vertex) {
              sol_obj.push_back_edge_to_route(train.name, e);
              current_vertex = instance.const_n().get_edge(e).target;
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
    const auto train = instance.get_train_list().get_train(tr);
    for (size_t t = train_interval[tr].first;
         t <= train_interval[tr].second + 1; ++t) {
      const auto train_speed_val =
          round_to(vars.at("v").at(tr, t).get(GRB_DoubleAttr_X), V_MIN);
      sol_obj.add_train_speed(tr, static_cast<int>(t) * dt, train_speed_val);
    }
  }

  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto  train  = instance.get_train_list().get_train(tr);
    const auto& tr_len = train.length;
    const auto& r_len  = sol_obj.get_instance().route_length(train.name);
    for (auto t = train_interval[tr].first; t <= train_interval[tr].second;
         ++t) {
      double train_pos = r_len;
      if (fix_routes) {
        train_pos = vars.at("lda").at(tr, t).get(GRB_DoubleAttr_X);
      } else {
        const double len_in =
            round_to(vars.at("len_in").at(tr, t).get(GRB_DoubleAttr_X),
                     ROUNDING_PRECISION);
        if (len_in > EPS) {
          train_pos = -len_in;
        } else {
          for (auto e_index :
               sol_obj.get_instance().get_route(train.name).get_edges()) {
            const bool e_used =
                vars.at("x").at(tr, t, e_index).get(GRB_DoubleAttr_X) > 0.5;
            if (e_used) {
              const double lda_val =
                  vars.at("e_lda").at(tr, t, e_index).get(GRB_DoubleAttr_X);
              const double e_pos = sol_obj.get_instance()
                                       .route_edge_pos(train.name, e_index)
                                       .first;
              if (lda_val + e_pos < train_pos) {
                train_pos = lda_val + e_pos;
              }
            }
          }
        }
      }

      train_pos += tr_len;
      train_pos = round_to(train_pos, ROUNDING_PRECISION);
      sol_obj.add_train_pos(tr, static_cast<int>(t) * dt, train_pos);
    }

    auto   t_final         = train_interval[tr].second + 1;
    double train_pos_final = -1;
    if (fix_routes) {
      train_pos_final =
          round_to(vars.at("mu").at(tr, t_final - 1).get(GRB_DoubleAttr_X),
                   ROUNDING_PRECISION);
    } else {
      train_pos_final =
          r_len +
          round_to(vars.at("len_out").at(tr, t_final - 1).get(GRB_DoubleAttr_X),
                   ROUNDING_PRECISION);
    }
    if (include_braking_curves) {
      train_pos_final -= round_to(
          vars.at("brakelen").at(tr, t_final - 1).get(GRB_DoubleAttr_X),
          ROUNDING_PRECISION);
    }
    train_pos_final = round_to(train_pos_final, ROUNDING_PRECISION);
    sol_obj.add_train_pos(tr, static_cast<int>(t_final) * dt, train_pos_final);
  }

  return sol_obj;
}

// NOLINTEND(performance-inefficient-string-concatenation)

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
