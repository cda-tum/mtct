#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "GeneralHelper.hpp"

#include <algorithm>
#include <cstddef>

// ---------------------
// INSTANCE
// ---------------------

cda_rail::instances::GeneralPerformanceOptimizationInstance::
    GeneralPerformanceOptimizationInstance(
        std::string_view const       instanceName,
        std::string_view const       instanceSubdirectory,
        std::filesystem::path const& workingDirectory)
    : GeneralProblemInstanceWithScheduleAndRoutes(
          instanceName, instanceSubdirectory, workingDirectory) {
  initialize_vectors();

  std::ifstream file(workingDirectory / "instances" / instanceSubdirectory /
                     instanceName / "problem_data.json");
  json          j = json::parse(file);
  for (const auto& [train_name, weight] : j["train_weights"].items()) {
    set_train_weight(train_name, static_cast<double>(weight));
  }
  for (const auto& [train_name, optional] : j["train_optional"].items()) {
    set_train_optionality_value(train_name, static_cast<bool>(optional));
  }
  m_station_delay_weight = static_cast<double>(j["station_delay_weight"]);
  m_lambda               = static_cast<double>(j["lambda"]);
}

double
cda_rail::instances::GeneralPerformanceOptimizationInstance::get_objective_val(
    const std::vector<double>&              tr_exit_times,
    const std::vector<std::vector<double>>& stop_times,
    const bool throw_error_if_not_all_stops_specified) const {
  double obj = 0.0;
  for (size_t tr = 0; tr < get_const_train_list().size(); ++tr) {
    auto const& tr_schedule = get_const_schedule(tr);

    obj += get_train_weight(tr) * tr_exit_times.at(tr);

    auto const& tr_stop_times   = stop_times.at(tr);
    auto const& scheduled_stops = tr_schedule.get_stops();
    if (scheduled_stops.size() < tr_stop_times.size()) {
      throw exceptions::InvalidInputException(concatenate_string_views(
          {"Number of stop times provided for train ",
           get_const_train_list().get_train(tr).get_name(),
           " is greater than the number of scheduled stops (",
           std::to_string(tr_stop_times.size()), " vs. ",
           std::to_string(scheduled_stops.size()), ")."}));
    }
    if (throw_error_if_not_all_stops_specified &&
        scheduled_stops.size() != tr_stop_times.size()) {
      throw exceptions::InvalidInputException(concatenate_string_views(
          {"Number of stop times provided for train ",
           get_const_train_list().get_train(tr).get_name(),
           " is not equal to the number of scheduled stops (",
           std::to_string(tr_stop_times.size()), " vs. ",
           std::to_string(scheduled_stops.size()), ")."}));
    }
    double delay_sum = 0.0;
    for (size_t stop_idx = 0; stop_idx < tr_stop_times.size(); ++stop_idx) {
      delay_sum += relu(tr_stop_times.at(stop_idx) -
                        scheduled_stops.at(stop_idx).get_service_time());
    }
    obj += get_station_delay_weight() * get_train_weight(tr) * delay_sum /
           static_cast<double>(scheduled_stops.size());
  }
  return obj;
}

void cda_rail::instances::GeneralPerformanceOptimizationInstance::
    export_instance(const std::filesystem::path& workingDirectory,
                    bool const                   saveNetwork) const {
  GeneralProblemInstanceWithScheduleAndRoutes::export_instance(workingDirectory,
                                                               saveNetwork);

  json j;
  for (size_t i = 0; i < m_train_weights.size(); ++i) {
    j["train_weights"][this->get_const_train_list().get_train(i).get_name()] =
        m_train_weights.at(i);
    j["train_optional"][this->get_const_train_list().get_train(i).get_name()] =
        m_train_optional.at(i);
  }
  j["station_delay_weight"] = m_station_delay_weight;
  j["lambda"]               = m_lambda;

  std::ofstream file(workingDirectory / "instances" /
                     get_instance_subdirectory() / get_instance_name() /
                     "problem_data.json");
  file << j << '\n';
}

void cda_rail::instances::GeneralPerformanceOptimizationInstance::
    discretize_stops() {
  /**
   * This method discretizes the network within the stations. It updates the
   * timetable and the routes accordingly.
   */

  for (const auto& station_name :
       this->get_const_station_list().get_station_names()) {
    const auto& station_tracks =
        this->get_const_station_list().get_station(station_name).tracks;
    const auto new_edges =
        this->get_editable_network().separate_stop_edges(station_tracks);
    this->get_editable_timetable().update_after_discretization(new_edges);
    this->get_editable_routes().update_after_discretization(new_edges);
  }
}

bool cda_rail::instances::GeneralPerformanceOptimizationInstance::
    check_consistency(bool every_train_must_have_route) const {
  if (!GeneralProblemInstanceWithScheduleAndRoutes::check_consistency(
          every_train_must_have_route)) {
    return false;
  }
  const auto num_tr = this->get_const_timetable().get_train_list().size();
  if (get_train_weights().size() != num_tr ||
      get_train_optional().size() != num_tr) {
    return false;
  }
  return (get_lambda() >= 0 && get_station_delay_weight() >= 0 &&
          std::ranges::all_of(get_train_weights(),
                              [](double w) { return w >= 0; }));
}

double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_approximate_leaving_time(size_t train) const {
  const auto& tr_object = this->get_const_train_list().get_train(train);
  const auto& timetable = this->get_const_timetable().get_schedule(train);
  return tr_object.get_length() / timetable.get_exit_velocity();
}

double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_maximal_leaving_time(size_t train, double v) const {
  const auto& tr_object = this->get_const_train_list().get_train(train);
  const auto& timetable = this->get_const_timetable().get_schedule(train);
  return cda_rail::max_travel_time_no_stopping(
      v, timetable.get_exit_velocity(), V_MIN, tr_object.get_acceleration(),
      tr_object.get_deceleration(), tr_object.get_length());
}

double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_minimal_leaving_time(size_t train, double v) const {
  const auto& tr_object = this->get_const_train_list().get_train(train);
  const auto& timetable = this->get_const_timetable().get_schedule(train);
  auto        v_max     = std::max(v, timetable.get_exit_velocity());
  if (v_max <= 0) {
    const auto& exit_node =
        this->get_const_timetable().get_schedule(train).get_exit_vertex();
    auto const& in_edges = this->get_const_network().in_edges(exit_node);
    assert(in_edges.size() == 1);
    const auto exit_edge = *in_edges.begin();
    v_max = this->get_const_network().get_edge(exit_edge).max_speed;
  }
  v_max = std::min(v_max, tr_object.get_max_speed());
  return cda_rail::min_travel_time(
      v, timetable.get_exit_velocity(), v_max, tr_object.get_acceleration(),
      tr_object.get_deceleration(), tr_object.get_length());
}

// -----------------
// SOLUTION
// -----------------

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    initialize_vectors() {
  auto const& train_list = this->get_instance()->get_const_train_list();
  m_train_pos.reserve(train_list.size());
  m_train_speed.reserve(train_list.size());
  m_train_routed = std::vector<bool>(train_list.size(), false);
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    m_train_pos.emplace_back();
    m_train_speed.emplace_back();
  }
}

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    load_solution(const std::filesystem::path& workingDirectory,
                  std::string_view const       solutionSubdirectory) {
  SolGeneralProblemInstanceWithScheduleAndRoutes::load_solution(
      workingDirectory, solutionSubdirectory);

  const auto p = get_export_path(workingDirectory, solutionSubdirectory);
  // Read train_pos
  std::ifstream train_pos_file(p / "train_pos.json");
  json          train_pos_json = json::parse(train_pos_file);
  for (const auto& [tr_name, tr_pos_json] : train_pos_json.items()) {
    for (const auto& [idx, pos_pair] : tr_pos_json.items()) {
      const auto [t, pos] = pos_pair.template get<std::pair<double, double>>();
      this->add_train_pos(tr_name, t, pos);
    }
  }

  // Read train_speed-
  std::ifstream train_speed_file(p / "train_speed.json");
  json          train_speed_json = json::parse(train_speed_file);
  for (const auto& [tr_name, tr_speed_json] : train_speed_json.items()) {
    for (const auto& [idx, speed_pair] : tr_speed_json.items()) {
      const auto [t, speed] =
          speed_pair.template get<std::pair<double, double>>();
      this->add_train_speed(tr_name, t, speed);
    }
  }

  // Read train_routed
  std::ifstream train_routed_file(p / "train_routed.json");
  json          train_routed_json = json::parse(train_routed_file);
  for (const auto& [tr_name, routed] : train_routed_json.items()) {
    this->set_train_routed_value(tr_name, routed.get<bool>());
  }
}

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    export_solution(const std::filesystem::path& workingDirectory,
                    std::string_view const       solutionSubdirectory,
                    bool                         export_instance) const {
  /**
   * This method exports the solution object to a specific path. This includes
   * the following:
   * - If export_instance is true, the instance is exported
   * - dt, status, obj, and postprocessed are exported to p / solution /
   * data.json
   * - train_pos and train_speed are exported
   *
   * The method throws a
   * ConsistencyException if the solution is not consistent.
   *
   */

  if (!check_consistency()) {
    throw exceptions::ConsistencyException();
  }

  SolGeneralProblemInstanceWithScheduleAndRoutes::export_solution(
      workingDirectory, solutionSubdirectory);

  std::filesystem::path const p =
      get_export_path(workingDirectory, solutionSubdirectory);

  // NOLINTBEGIN(misc-const-correctness)
  json train_pos_json;
  json train_speed_json;
  json train_routed_json;
  // NOLINTEND(misc-const-correctness)
  auto const& train_list = this->get_instance()->get_const_train_list();
  for (size_t tr_id = 0; tr_id < train_list.size(); ++tr_id) {
    const auto& train                   = train_list.get_train(tr_id);
    train_pos_json[train.get_name()]    = m_train_pos.at(tr_id);
    train_speed_json[train.get_name()]  = m_train_speed.at(tr_id);
    train_routed_json[train.get_name()] = m_train_routed.at(tr_id);
  }

  std::ofstream train_pos_file(p / "train_pos.json");
  train_pos_file << train_pos_json << '\n';
  train_pos_file.close();

  std::ofstream train_speed_file(p / "train_speed.json");
  train_speed_file << train_speed_json << '\n';
  train_speed_file.close();

  std::ofstream train_routed_file(p / "train_routed.json");
  train_routed_file << train_routed_json << '\n';
  train_routed_file.close();
}

double
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::get_train_pos(
    const std::string& tr_name, double t) const {
  auto const& train_list = this->get_instance()->get_const_train_list();

  if (!train_list.has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  const auto tr_id = train_list.get_train_index(tr_name);
  if (m_train_pos.at(tr_id).contains(t)) {
    return m_train_pos.at(tr_id).at(t);
  }
  throw exceptions::ConsistencyException("No position for train " + tr_name +
                                         " at time " + std::to_string(t));
}

cda_rail::instances::SolGeneralPerformanceOptimizationInstance::EdgeTimeBound
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    get_edge_and_time_bounds(const std::string& tr_name, double t) const {
  auto const& train_list = this->get_instance()->get_const_train_list();
  if (!train_list.has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  const auto  tr_id  = train_list.get_train_index(tr_name);
  const auto& tr_pos = m_train_pos.at(tr_id);

  double t0 = -1;
  double t1 = -1;
  for (const auto& [time, pos] : tr_pos) {
    if (std::abs(time - t) < GRB_EPS) {
      t0 = time;
      t1 = time;
      break;
    }
    if (time < t) {
      t0 = time;
    } else if (t0 >= 0) {
      t1 = time;
      break;
    } else {
      throw exceptions::ConsistencyException(
          "Train " + tr_name + " not present at time " + std::to_string(t));
    }
  }
  if (t1 < 0) {
    throw exceptions::ConsistencyException(
        "Train " + tr_name + " not present at time " + std::to_string(t));
  }

  assert(t >= t0 - GRB_EPS);
  assert(t <= t1 + GRB_EPS);
  const auto pos0 = get_train_pos(tr_name, t0);

  const Network& n = this->get_instance()->get_const_network();
  const auto     r_len =
      this->get_const_solution_routes().get_route(tr_name).length(n);
  return {
      this->get_const_solution_routes().get_route(tr_name).get_edge_id_at_pos(
          std::min(pos0 + GRB_EPS, r_len), n),
      t0, t1};
}

cda_rail::instances::SolGeneralPerformanceOptimizationInstance::PosVelBound
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    get_exact_pos_and_vel_bounds(const std::string& tr_name, double t) const {
  const auto [edge, t1, t2] = get_edge_and_time_bounds(tr_name, t);
  assert(t >= t1 - GRB_EPS);
  assert(t <= t2 + GRB_EPS);

  const auto v1   = get_train_speed(tr_name, t1);
  const auto v2   = get_train_speed(tr_name, t2);
  const auto pos1 = get_train_pos(tr_name, t1);
  const auto pos2 = get_train_pos(tr_name, t2);

  const Route& tr_route = this->get_const_solution_routes().get_route(tr_name);
  const auto&  r_len =
      tr_route.length(this->get_instance()->get_const_network());
  const bool tr_leaving_route = pos2 >= r_len + GRB_EPS;

  if (std::abs(pos2 - pos1) < GRB_EPS) {
    return {std::min(pos1, pos2), std::max(pos1, pos2), std::min(v1, v2),
            std::max(v1, v2)};
  }

  const auto& edge_obj =
      this->get_instance()->get_const_network().get_edge(edge);
  const auto& tr_obj =
      this->get_instance()->get_const_train_list().get_train(tr_name);

  const auto max_speed =
      tr_leaving_route ? tr_obj.get_max_speed()
                       : std::min(edge_obj.max_speed, tr_obj.get_max_speed());

  const auto max_t = max_travel_time(v1, v2, V_MIN, tr_obj.get_acceleration(),
                                     tr_obj.get_deceleration(), pos2 - pos1,
                                     edge_obj.breakable);
  const auto min_t =
      min_travel_time(v1, v2, max_speed, tr_obj.get_acceleration(),
                      tr_obj.get_deceleration(), pos2 - pos1);
  double ub   = pos1;
  double lb   = pos1;
  double v_lb = 0;
  double v_ub = max_speed;

  if (max_t >= std::numeric_limits<double>::infinity()) {
    const auto t_to_stop = v1 / tr_obj.get_deceleration();
    const auto rel_t     = std::min(t_to_stop, t - t1);
    lb += v1 * rel_t - 0.5 * tr_obj.get_deceleration() * rel_t * rel_t;
    v_lb = v1 - tr_obj.get_deceleration() * rel_t;
  } else {
    const auto min_speed =
        minimal_line_speed(v1, v2, V_MIN, tr_obj.get_acceleration(),
                           tr_obj.get_deceleration(), pos2 - pos1);
    lb += pos_on_edge_at_time(v1, v2, min_speed, tr_obj.get_acceleration(),
                              tr_obj.get_deceleration(), pos2 - pos1, t - t1);
    v_lb = vel_on_edge_at_time(v1, v2, min_speed, tr_obj.get_acceleration(),
                               tr_obj.get_deceleration(), pos2 - pos1, t - t1);
  }

  if (t >= t1 + min_t) {
    ub += pos2 - pos1;
  } else {
    const auto max_line_speed =
        maximal_line_speed(v1, v2, max_speed, tr_obj.get_acceleration(),
                           tr_obj.get_deceleration(), pos2 - pos1);
    ub += pos_on_edge_at_time(v1, v2, max_line_speed, tr_obj.get_acceleration(),
                              tr_obj.get_deceleration(), pos2 - pos1, t - t1);
    v_ub =
        vel_on_edge_at_time(v1, v2, max_line_speed, tr_obj.get_acceleration(),
                            tr_obj.get_deceleration(), pos2 - pos1, t - t1);
  }
  return {{lb, ub}, {v_lb, v_ub}};
}

std::optional<
    cda_rail::instances::SolGeneralPerformanceOptimizationInstance::PosVel>
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    get_approximate_train_pos_and_vel(const std::string& tr_name,
                                      double             t) const {
  const auto [edge, t1, t2] = get_edge_and_time_bounds(tr_name, t);
  assert(t >= t1 - GRB_EPS);
  assert(t <= t2 + GRB_EPS);

  const auto pos_1 = get_train_pos(tr_name, t1);
  const auto v1    = get_train_speed(tr_name, t1);

  if (t1 == t2) {
    return {{pos_1, v1}};
  }

  const auto pos_2 = get_train_pos(tr_name, t2);
  const auto v2    = get_train_speed(tr_name, t2);

  const auto& edge_obj =
      this->get_instance()->get_const_network().get_edge(edge);
  const auto& tr_obj =
      this->get_instance()->get_const_train_list().get_train(tr_name);
  const auto max_speed = std::min(tr_obj.get_max_speed(), edge_obj.max_speed);
  const auto dist_travelled = pos_2 - pos_1;

  if (std::abs(dist_travelled) < GRB_EPS) {
    // Train stopped
    return {{pos_1, 0}};
  }

  const auto v_line =
      get_line_speed(v1, v2, V_MIN, max_speed, tr_obj.get_acceleration(),
                     tr_obj.get_deceleration(), dist_travelled, t2 - t1);
  if (v_line <= 0) {
    return std::nullopt;
  }

  const auto tr_pos =
      get_train_pos(tr_name, t1) +
      pos_on_edge_at_time(v1, v2, v_line, tr_obj.get_acceleration(),
                          tr_obj.get_deceleration(), dist_travelled, t - t1);
  const auto tr_vel =
      vel_on_edge_at_time(v1, v2, v_line, tr_obj.get_acceleration(),
                          tr_obj.get_deceleration(), dist_travelled, t - t1);

  return {{tr_pos, tr_vel}};
}

double
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::get_train_speed(
    const std::string& tr_name, double t) const {
  auto const& train_list = this->get_instance()->get_const_train_list();

  if (!train_list.has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  const auto tr_id = train_list.get_train_index(tr_name);
  if (m_train_speed.at(tr_id).contains(t)) {
    return m_train_speed.at(tr_id).at(t);
  }
  throw exceptions::ConsistencyException("No speed for train " + tr_name +
                                         " at time " + std::to_string(t));
}

bool cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    get_train_routed(const std::string& tr_name) const {
  auto const& train_list = this->get_instance()->get_const_train_list();

  if (!train_list.has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  return m_train_routed.at(train_list.get_train_index(tr_name));
}

std::vector<double>
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::get_train_times(
    const std::string& tr_name) const {
  auto const& train_list = this->get_instance()->get_const_train_list();

  if (!train_list.has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  const auto  tr_id        = train_list.get_train_index(tr_name);
  const auto& tr_speed_map = m_train_speed.at(tr_id);
  // Return keys of the map
  std::vector<double> times;
  for (const auto& [t, _] : tr_speed_map) {
    times.push_back(t);
  }
  // Sort
  std::sort(times.begin(), times.end());
  return times;
}

cda_rail::index_vector
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::get_train_order(
    size_t edge_index) const {
  cda_rail::index_set tr_on_edge = this->get_instance()->all_trains_on_edge(
      get_const_solution_routes(), edge_index);

  std::map<size_t, double> tr_times;
  for (const auto& tr : tr_on_edge) {
    const Train& tr_object =
        this->get_instance()->get_const_train_list().get_train(tr);
    const double e_pos_source =
        this->get_const_solution_routes()
            .get_route(tr_object.get_name())
            .edge_pos_on_route(edge_index,
                               this->get_instance()->get_const_network())
            .source;

    const auto time_at_e_pos_source =
        get_time_at_pos(tr_object.get_name(), e_pos_source);
    tr_times.insert({tr, time_at_e_pos_source});
  }

  // Create a vector of tr_on_edge (set) ordered by tr_times
  cda_rail::index_vector tr_on_edge_vec(tr_on_edge.size());
  std::ranges::move(tr_on_edge, tr_on_edge_vec.begin());
  std::ranges::sort(tr_on_edge_vec, [&tr_times](size_t tr1, size_t tr2) {
    return tr_times.at(tr1) < tr_times.at(tr2);
  });
  return tr_on_edge_vec;
}

std::vector<cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
                TrainDirection>
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    get_train_order_with_reverse(size_t edge_index) const {
  cda_rail::index_set tr_on_edge = this->get_instance()->all_trains_on_edge(
      get_const_solution_routes(), edge_index);
  const std::optional<size_t> rev_e =
      this->get_instance()->get_const_network().get_reverse_edge_index(
          edge_index);
  cda_rail::index_set tr_on_rev_edge;
  if (rev_e.has_value()) {
    tr_on_rev_edge = this->get_instance()->all_trains_on_edge(
        get_const_solution_routes(), rev_e.value());
  }

  std::vector<TrainDirection> ret_vec;
  std::map<size_t, double>    tr_times;
  for (size_t i = 0; i < (rev_e.has_value() ? 2 : 1); ++i) {
    const bool  direction   = i == 0;
    const auto& rel_e       = direction ? edge_index : rev_e.value();
    const auto& rel_tr_on_e = direction ? tr_on_edge : tr_on_rev_edge;
    for (const auto& tr : rel_tr_on_e) {
      const Train& tr_object =
          this->get_instance()->get_const_train_list().get_train(tr);
      const double e_pos_source =
          this->get_const_solution_routes()
              .get_route(tr_object.get_name())
              .edge_pos_on_route(rel_e,
                                 this->get_instance()->get_const_network())
              .source;
      const auto time_at_e_pos_source =
          get_time_at_pos(tr_object.get_name(), e_pos_source);
      tr_times.insert({tr, time_at_e_pos_source});
      ret_vec.emplace_back(tr, direction);
    }
  }

  std::ranges::sort(
      ret_vec, [&tr_times](TrainDirection tr1, TrainDirection tr2) {
        return tr_times.at(tr1.train_id) < tr_times.at(tr2.train_id);
      });

  return ret_vec;
}

double
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::get_time_at_pos(
    const std::string& tr_name, double pos) const {
  if (!this->get_instance()->get_const_train_list().has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  const auto tr_times = get_train_times(tr_name);
  for (const auto& t : tr_times) {
    if (std::abs(get_train_pos(tr_name, t) - pos) < GRB_EPS) {
      return t;
    }
  }
  throw exceptions::ConsistencyException("No time for train " + tr_name +
                                         " at position " + std::to_string(pos));
}

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    add_train_pos(const std::string& tr_name, double t, double pos) {
  auto const& train_list = this->get_instance()->get_const_train_list();

  if (!train_list.has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  if (pos + EPS < 0) {
    throw exceptions::ConsistencyException("Position must be non-negative");
  }
  if (t + EPS < 0) {
    throw exceptions::ConsistencyException("Time must be non-negative");
  }

  const auto tr_id = train_list.get_train_index(tr_name);
  if (m_train_pos.at(tr_id).contains(t)) {
    m_train_pos.at(tr_id).at(t) = pos;
  } else {
    m_train_pos.at(tr_id).insert({t, pos});
  }
}

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    add_train_speed(const std::string& tr_name, double t, double speed) {
  auto const& train_list = this->get_instance()->get_const_train_list();

  if (!train_list.has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  if (speed + EPS < 0) {
    throw exceptions::ConsistencyException("Speed must be non-negative");
  }
  if (t + EPS < 0) {
    throw exceptions::ConsistencyException("Time must be non-negative");
  }

  const auto tr_id = train_list.get_train_index(tr_name);
  if (m_train_speed.at(tr_id).contains(t)) {
    m_train_speed.at(tr_id).at(t) = speed;
  } else {
    m_train_speed.at(tr_id).insert({t, speed});
  }
}

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    set_train_routed_value(const std::string& tr_name, bool val) {
  auto const& train_list = this->get_instance()->get_const_train_list();

  if (!train_list.has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  m_train_routed.at(train_list.get_train_index(tr_name)) = val;
}

bool cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    check_consistency() const {
  if (!SolGeneralProblemInstanceWithScheduleAndRoutes::check_consistency()) {
    return false;
  }

  if (!this->has_solution()) {
    return true;
  }

  for (auto tr_id = 0; tr_id < m_train_routed.size(); tr_id++) {
    const auto& tr_name = this->get_instance()
                              ->get_const_train_list()
                              .get_train(tr_id)
                              .get_name();
    if (m_train_routed.at(tr_id) &&
        !get_const_solution_routes().has_route(tr_name)) {
      return false;
    }
    if (!m_train_routed.at(tr_id) &&
        !this->get_instance()->get_train_optional().at(tr_id)) {
      return false;
    }
    if (m_train_routed.at(tr_id) && m_train_pos.at(tr_id).size() < 2) {
      // At least two points of information are needed to recover the timing
      return false;
    }

    if (m_train_pos.size() != m_train_speed.size()) {
      return false;
    }

    for (const auto& t : m_train_pos.at(tr_id) | std::views::keys) {
      if (m_train_speed.at(tr_id).count(t) != 1) {
        return false;
      }
    }
  }

  for (const auto& train_pos_vec : m_train_pos) {
    for (const auto& pos : train_pos_vec | std::views::values) {
      if (pos + EPS < 0) {
        return false;
      }
    }
  }
  for (size_t tr_id = 0; tr_id < m_train_speed.size(); ++tr_id) {
    const auto& train =
        this->get_instance()->get_const_train_list().get_train(tr_id);
    for (const auto& v : m_train_speed.at(tr_id) | std::views::values) {
      if (v + EPS < 0 || v > train.get_max_speed() + EPS) {
        return false;
      }
    }
  }
  return true;
}

// --------------------
// SolVSSGeneralPerformanceOptimizationInstance
// --------------------

void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    initialize_vss_vector() {
  m_vss_pos = std::vector<std::vector<double>>(
      this->get_instance()->get_const_network().number_of_edges());
}

void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    add_vss_pos(cda_rail::Network::EdgeInput const& edge_input, double pos,
                bool reverse_edge) {
  // Add VSS position on edge. Also on reverse edge if true.

  auto const edge_id =
      this->get_instance()->get_const_network().get_edge_index(edge_input);
  auto const& edge =
      this->get_instance()->get_const_network().get_edge(edge_id);

  if (pos <= EPS || pos + EPS >= edge.length) {
    throw exceptions::ConsistencyException(
        "VSS position " + std::to_string(pos) + " is not on edge " +
        std::to_string(edge_id));
  }

  m_vss_pos.at(edge_id).emplace_back(pos);
  std::ranges::sort(m_vss_pos.at(edge_id));

  if (reverse_edge) {
    const auto reverse_edge_index =
        this->get_instance()->get_const_network().get_reverse_edge_index(
            edge_id);
    if (reverse_edge_index.has_value()) {
      m_vss_pos.at(reverse_edge_index.value()).emplace_back(edge.length - pos);
      std::ranges::sort(m_vss_pos.at(reverse_edge_index.value()));
    }
  }
}

void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    set_vss_pos(cda_rail::Network::EdgeInput const& edge_input,
                std::vector<double>                 pos) {
  auto const edge_id =
      this->get_instance()->get_const_network().get_edge_index(edge_input);
  auto const& edge =
      this->get_instance()->get_const_network().get_edge(edge_id);

  for (const auto& p : pos) {
    if (p <= EPS || p + EPS >= edge.length) {
      throw exceptions::ConsistencyException(
          "VSS position " + std::to_string(p) + " is not on edge " +
          std::to_string(edge_id));
    }
  }

  std::ranges::sort(pos);
  m_vss_pos.at(edge_id) = std::move(pos);
}

void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    reset_vss_pos(cda_rail::Network::EdgeInput const& edge_input) {
  auto const edge_id =
      this->get_instance()->get_const_network().get_edge_index(edge_input);
  m_vss_pos.at(edge_id).clear();
}

void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    export_solution(const std::filesystem::path& workingDirectory,
                    std::string_view const       solutionSubdirectory,
                    bool                         export_instance) const {
  SolGeneralPerformanceOptimizationInstance::export_solution(
      workingDirectory, solutionSubdirectory, export_instance);

  // NOLINTNEXTLINE(misc-const-correctness)
  json vss_pos_json;

  auto const& const_network = this->get_instance()->get_const_network();
  for (size_t edge_id = 0; edge_id < const_network.number_of_edges();
       ++edge_id) {
    const auto& edge = const_network.get_edge(edge_id);
    const auto& v0   = const_network.get_vertex(edge.source).name;
    const auto& v1   = const_network.get_vertex(edge.target).name;
    vss_pos_json[const_network.get_edge_name(edge_id)] = m_vss_pos.at(edge_id);
  }

  std::filesystem::path const p =
      get_export_path(workingDirectory, solutionSubdirectory);
  std::ofstream vss_pos_file(p / "vss_pos.json");
  vss_pos_file << vss_pos_json << '\n';
  vss_pos_file.close();
}

bool cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    check_consistency() const {
  if (!SolGeneralPerformanceOptimizationInstance::check_consistency()) {
    return false;
  }

  auto const& const_network = this->get_instance()->get_const_network();
  for (size_t edge_id = 0; edge_id < m_vss_pos.size(); ++edge_id) {
    const auto& edge = const_network.get_edge(edge_id);
    if (!edge.breakable && !m_vss_pos.at(edge_id).empty()) {
      return false;
    }
    for (const auto& pos : m_vss_pos.at(edge_id)) {
      if (pos + EPS < 0 || pos > edge.length + EPS) {
        return false;
      }
    }
  }
  return true;
}

void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    load_solution(const std::filesystem::path& workingDirectory,
                  std::string_view const       solutionSubdirectory) {
  SolGeneralPerformanceOptimizationInstance::load_solution(
      workingDirectory, solutionSubdirectory);

  std::filesystem::path const p =
      this->get_export_path(workingDirectory, solutionSubdirectory);
  std::ifstream vss_pos_file(p / "vss_pos.json");
  json          vss_pos_json = json::parse(vss_pos_file);
  vss_pos_file.close();

  auto const& const_network = this->get_instance()->get_const_network();
  for (const auto& [edge_str, pos_vec_json] : vss_pos_json.items()) {
    const auto          edge_id = const_network.get_edge_index({edge_str});
    std::vector<double> pos_vec = pos_vec_json.get<std::vector<double>>();
    std::ranges::sort(pos_vec);
    m_vss_pos.at(edge_id) = std::move(pos_vec);
  }
}
