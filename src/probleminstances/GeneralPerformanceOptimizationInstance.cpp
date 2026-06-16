#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "GeneralHelper.hpp"
#include "StringHelper.hpp"
#include "datastructure/Timetable.hpp"
#include "nlohmann/json.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// ---------------------
// INSTANCE
/**
 * @brief Initializes the instance with train weights and station delay weight loaded from a configuration file.
 *
 * @param instanceName Name of the instance to load.
 * @param instanceSubdirectory Subdirectory containing the instance.
 * @param working_directory Root working directory.
 */

cda_rail::instances::GeneralPerformanceOptimizationInstance::
    GeneralPerformanceOptimizationInstance(
        std::string_view const       instanceName,
        std::string_view const       instanceSubdirectory,
        std::filesystem::path const& working_directory)
    : GeneralProblemInstanceWithScheduleAndRoutes(
          instanceName, instanceSubdirectory, working_directory) {
  initialize_vectors();

  std::ifstream file(working_directory / "instances" / instanceSubdirectory /
                     instanceName / "problem_data.json");
  json          j = json::parse(file);
  for (const auto& [train_name, weight] : j.at("train_weights").items()) {
    set_train_weight(train_name, static_cast<double>(weight));
  }
  m_station_delay_weight = static_cast<double>(j.at("station_delay_weight"));
}

/**
 * @brief Computes a weighted objective combining train exit times and station delays.
 *
 * The objective sums (train weight × exit time) for each train and adds a weighted
 * penalty for positive deviations between provided and scheduled stop times.
 * Each train's delay penalty is (station_delay_weight × train_weight × average_delay) / scheduled_stop_count.
 *
 * @param tr_exit_times Exit times for each train.
 * @param stop_times Arrival times at scheduled stops; one vector per train. May be partial.
 * @param throwErrorIfNotAllStopsSpecified If true, requires all scheduled stops to be specified.
 * @return double The computed objective value.
 * @throws InvalidInputException if input sizes don't match the train count, or stop-time counts are invalid.
 */
double
cda_rail::instances::GeneralPerformanceOptimizationInstance::get_objective_val(
    const std::vector<double>&              tr_exit_times,
    const std::vector<std::vector<double>>& stop_times,
    const bool throwErrorIfNotAllStopsSpecified) const {
  const auto train_count = get_const_train_list().size();
  if (tr_exit_times.size() != train_count || stop_times.size() != train_count) {
    throw exceptions::InvalidInputException(
        "Objective input vector sizes must match the number of trains.");
  }

  double obj = 0.0;
  for (size_t tr = 0; tr < train_count; ++tr) {
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
    if (throwErrorIfNotAllStopsSpecified &&
        scheduled_stops.size() != tr_stop_times.size()) {
      throw exceptions::InvalidInputException(concatenate_string_views(
          {"Number of stop times provided for train ",
           get_const_train_list().get_train(tr).get_name(),
           " is not equal to the number of scheduled stops (",
           std::to_string(tr_stop_times.size()), " vs. ",
           std::to_string(scheduled_stops.size()), ")."}));
    }

    if (!tr_stop_times.empty()) {
      double delay_sum = 0.0;
      for (size_t stop_idx = 0; stop_idx < tr_stop_times.size(); ++stop_idx) {
        delay_sum += relu(tr_stop_times.at(stop_idx) -
                          scheduled_stops.at(stop_idx).get_service_time());
      }
      obj += get_station_delay_weight() * get_train_weight(tr) * delay_sum /
             static_cast<double>(scheduled_stops.size());
    }
  }
  return obj;
}

/**
 * @brief Persists instance configuration to disk.
 *
 * Writes train weights and station delay weight, and delegates to the base 
 * class to export the schedule and routes.
 *
 * @param working_directory Base output directory for the instance files.
 * @param saveNetwork       If true, also exports the network.
 */
void cda_rail::instances::GeneralPerformanceOptimizationInstance::
    export_instance(const std::filesystem::path& working_directory,
                    bool const                   saveNetwork) const {
  GeneralProblemInstanceWithScheduleAndRoutes::export_instance(
      working_directory, saveNetwork);
  // NOLINTBEGIN(*-pro-bounds-avoid-unchecked-container-access)
  json j;
  for (size_t i = 0; i < m_train_weights.size(); ++i) {
    j["train_weights"][this->get_const_train_list().get_train(i).get_name()] =
        m_train_weights.at(i);
  }
  j["station_delay_weight"] = m_station_delay_weight;
  // NOLINTEND(*-pro-bounds-avoid-unchecked-container-access)

  std::ofstream file(working_directory / "instances" /
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

/**
 * @brief Validates the consistency of the performance optimization instance.
 *
 * Verifies that the train weight vector size matches the timetable train count,
 * the station delay weight is non-negative, and all individual train weights are non-negative.
 * Also delegates to the base class consistency check.
 *
 * @param every_train_must_have_route Whether each train must have an assigned route.
 * @return `true` if all consistency checks pass, `false` otherwise.
 */
bool cda_rail::instances::GeneralPerformanceOptimizationInstance::
    check_consistency(bool every_train_must_have_route) const {
  if (!GeneralProblemInstanceWithScheduleAndRoutes::check_consistency(
          every_train_must_have_route)) {
    return false;
  }
  const auto num_tr = this->get_const_timetable().get_train_list().size();
  if (get_train_weights().size() != num_tr) {
    return false;
  }
  return (get_station_delay_weight() >= 0 &&
          std::ranges::all_of(get_train_weights(),
                              [](double w) { return w >= 0; }));
}

/**
 * @brief Computes the approximate time for a train to exit.
 *
 * @param train Train index.
 * @return Approximate exit time.
 */
double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_approximate_leaving_time(size_t train) const {
  const auto& tr_object = this->get_const_train_list().get_train(train);
  const auto& timetable = this->get_const_timetable().get_schedule(train);
  return tr_object.get_length() / timetable.get_exit_velocity();
}

/**
 * @brief Calculates the maximum time required for a train to exit.
 *
 * @param v Requested speed for the exit.
 * @return Maximum time for the train to exit.
 */
double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_maximal_leaving_time(size_t train, double v) const {
  const auto& tr_object = this->get_const_train_list().get_train(train);
  const auto& timetable = this->get_const_timetable().get_schedule(train);
  return cda_rail::max_travel_time_no_stopping(
      v, timetable.get_exit_velocity(), V_MIN, tr_object.get_acceleration(),
      tr_object.get_deceleration(), tr_object.get_length());
}

/**
 * @brief Computes the minimum time required for a train to exit the network given a target velocity.
 *
 * @param train Train index.
 * @param v Target velocity.
 * @return Minimum travel time in seconds for the train to reach the exit node.
 */
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
/**
 * @brief Initializes per-train storage containers for position and speed data.
 *
 * Reserves capacity and creates an empty container for each train to store position and speed values.
 */

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    initialize_vectors() {
  auto const& train_list = this->get_instance()->get_const_train_list();
  m_train_pos.reserve(train_list.size());
  m_train_speed.reserve(train_list.size());
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    m_train_pos.emplace_back();
    m_train_speed.emplace_back();
  }
}

/**
 * @brief Loads a solution from disk, including train position and speed trajectories.
 *
 * Reads `train_pos.json` and `train_speed.json` from the solution's export directory,
 * parsing time-indexed (time, value) pairs for each train and populating the solution's
 * internal position and speed maps.
 *
 * @param working_directory Root directory containing the solution export.
 * @param solutionSubdirectory Subdirectory name within the solutions folder.
 * @param parameter_identifier Optional identifier to distinguish solution variants.
 */
void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    load_solution(const std::filesystem::path&      working_directory,
                  std::string_view const            solutionSubdirectory,
                  std::optional<std::string> const& parameter_identifier) {
  SolGeneralProblemInstanceWithScheduleAndRoutes::load_solution(
      working_directory, solutionSubdirectory, parameter_identifier);

  const auto p = get_export_path(working_directory, solutionSubdirectory,
                                 parameter_identifier);
  // Read train_pos
  std::ifstream train_pos_file(p / "train_pos.json");
  json          train_pos_json = json::parse(train_pos_file);
  for (const auto& [tr_name, tr_pos_json] : train_pos_json.items()) {
    for (const auto& [idx, pos_pair] : tr_pos_json.items()) {
      const auto [t, pos] = pos_pair.get<std::pair<double, double>>();
      this->add_train_pos(tr_name, t, pos);
    }
  }

  // Read train_speed-
  std::ifstream train_speed_file(p / "train_speed.json");
  json          train_speed_json = json::parse(train_speed_file);
  for (const auto& [tr_name, tr_speed_json] : train_speed_json.items()) {
    for (const auto& [idx, speed_pair] : tr_speed_json.items()) {
      const auto [t, speed] = speed_pair.get<std::pair<double, double>>();
      this->add_train_speed(tr_name, t, speed);
    }
  }
}

/**
 * @brief Exports the solution's train trajectories (position and speed over time) to JSON files.
 *
 * Validates solution consistency before exporting. Delegates to the base class for instance 
 * and solution metadata export, then writes `train_pos.json` and `train_speed.json` 
 * containing per-train trajectory data.
 *
 * @throws ConsistencyException if the solution is not consistent.
 */
void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    export_solution(
        const std::filesystem::path& working_directory,
        std::string_view const solutionSubdirectory, bool save_instance,
        std::optional<std::string> const& parameter_identifier) const {
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
      working_directory, solutionSubdirectory, save_instance,
      parameter_identifier);

  std::filesystem::path const p = get_export_path(
      working_directory, solutionSubdirectory, parameter_identifier);

  // NOLINTBEGIN(misc-const-correctness)
  json train_pos_json;
  json train_speed_json;
  // NOLINTEND(misc-const-correctness)
  auto const& train_list = this->get_instance()->get_const_train_list();
  for (size_t tr_id = 0; tr_id < train_list.size(); ++tr_id) {
    const auto& train = train_list.get_train(tr_id);
    // NOLINTBEGIN(*-pro-bounds-avoid-unchecked-container-access)
    train_pos_json[train.get_name()]   = m_train_pos.at(tr_id);
    train_speed_json[train.get_name()] = m_train_speed.at(tr_id);
    // NOLINTEND(*-pro-bounds-avoid-unchecked-container-access)
  }

  std::ofstream train_pos_file(p / "train_pos.json");
  train_pos_file << train_pos_json << '\n';
  train_pos_file.close();

  std::ofstream train_speed_file(p / "train_speed.json");
  train_speed_file << train_speed_json << '\n';
  train_speed_file.close();
}

/**
 * @brief Retrieves the position of a train at a given time.
 *
 * @param tr_name Train name.
 * @param t Time at which to query position.
 * @return double The train's position at time `t`.
 *
 * @throws exceptions::TrainNotExistentException if the train does not exist.
 * @throws exceptions::ConsistencyException if no position data is available at the specified time.
 */
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

/**
 * @brief Determines the edge and surrounding time steps for a train at a given time.
 *
 * @param tr_name The name of the train.
 * @param t The time at which to query the train's edge.
 * @return EdgeTimeBound with the edge ID at the train's position and the bracketing time steps.
 *
 * @throws TrainNotExistentException if the train does not exist.
 * @throws ConsistencyException if no recorded position covers the given time.
 */
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
  for (const auto& time : tr_pos | std::ranges::views::keys) {
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
  return {.edge_id =
              this->get_const_solution_routes()
                  .get_route(tr_name)
                  .get_edge_id_at_pos(std::min(pos0 + GRB_EPS, r_len), n),
          .previous_time_step = t0,
          .next_time_step     = t1};
}

/**
 * @brief Computes position and velocity bounds for a train at a specified time.
 *
 * @param tr_name The train identifier.
 * @param t The time at which to compute bounds.
 * @return A `PosVelBound` struct with lower and upper bounds for position and velocity.
 */
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
    return {.pos = {.lb = std::min(pos1, pos2), .ub = std::max(pos1, pos2)},
            .vel = {.lb = std::min(v1, v2), .ub = std::max(v1, v2)}};
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
    lb += (v1 * rel_t) - (0.5 * tr_obj.get_deceleration() * rel_t * rel_t);
    v_lb = v1 - (tr_obj.get_deceleration() * rel_t);
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
  return {.pos = {.lb = lb, .ub = ub}, .vel = {.lb = v_lb, .ub = v_ub}};
}

/**
 * @brief Approximates the train's position and velocity at a given time.
 *
 * Interpolates position and velocity between the two nearest recorded time points 
 * surrounding the requested time. If the train did not move during the interval,
 * returns the stationary position with zero velocity.
 *
 * @return A struct containing `pos` and `vel`, or `std::nullopt` if the computed
 *         motion profile is infeasible.
 *
 * @throws exceptions::TrainNotExistentException If the train does not exist.
 * @throws exceptions::ConsistencyException If required time points are missing.
 */
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
    return {{.pos = pos_1, .vel = v1}};
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
    return {{.pos = pos_1, .vel = 0}};
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

  return {{.pos = tr_pos, .vel = tr_vel}};
}

/**
 * @brief Retrieves the speed of a train at a given time.
 *
 * @param tr_name Name of the train.
 * @param t Time at which to retrieve the speed.
 * @return double The train's speed at time t.
 *
 * @throws TrainNotExistentException if the train does not exist.
 * @throws ConsistencyException if no speed data is available for the train at time t.
 */
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

/**
 * @brief Retrieves all recorded times for a train's speed data.
 *
 * @return std::vector<double> Times in ascending order at which speed data was recorded for the train.
 * @throws exceptions::TrainNotExistentException if the train does not exist.
 */
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
  for (const auto& t : tr_speed_map | std::ranges::views::keys) {
    times.push_back(t);
  }
  // Sort
  std::ranges::sort(times);
  return times;
}

/**
 * @brief Orders trains present on an edge by the time they reached its source.
 *
 * @param edge_index The index of the network edge.
 * @return Vector of train indices sorted by the time each train reached the source position of the edge.
 */
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

/**
 * @brief Orders trains traversing an edge or its reverse edge by arrival time.
 *
 * @param edge_index The edge index to query.
 * @return A vector of TrainDirection entries sorted by the time each train reaches the edge's source position. The direction flag indicates forward traversal on the edge (true) or traversal via its reverse edge (false).
 */
std::vector<cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
                TrainDirection>
cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    get_train_order_with_reverse(size_t edge_index) const {
  cda_rail::index_set const tr_on_edge =
      this->get_instance()->all_trains_on_edge(get_const_solution_routes(),
                                               edge_index);
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
  for (size_t i = 0; i < (rev_e.has_value() ? 2U : 1U); ++i) {
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

/**
 * @brief Finds the time at which a train reaches a specified position.
 *
 * @param tr_name The name of the train.
 * @param pos The position along the route.
 * @return double The time at which the train is at the given position.
 *
 * @throws exceptions::TrainNotExistentException if the train does not exist.
 * @throws exceptions::ConsistencyException if no recorded time has the train at the given position.
 */
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

/**
 * @brief Records the position of a train at a given time.
 *
 * If a position already exists for the given time, it is overwritten; otherwise, a new entry is inserted.
 *
 * @param tr_name The train name.
 * @param t The time point.
 * @param pos The position on the route.
 *
 * @throws TrainNotExistentException If the train does not exist.
 * @throws ConsistencyException If the position or time is negative.
 */
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

/**
 * @brief Records or updates the speed of a train at a specific time.
 *
 * If an entry for time @p t already exists, its speed is overwritten; otherwise a new entry is created.
 *
 * @param tr_name Name of the train.
 * @param t Time at which to record the speed. Must be non-negative.
 * @param speed The speed value. Must be non-negative.
 *
 * @throws TrainNotExistentException If the train does not exist.
 * @throws ConsistencyException If speed or time is negative.
 */
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

/**
 * @brief Verifies the internal consistency of the solution data.
 *
 * Validates that all constraints are satisfied, including:
 * - Base class consistency.
 * - For each train with solution data: the train route is present, at least two time points are recorded
 *   (required to recover timing information), position and speed time keys are aligned, and all position
 *   and speed values are non-negative and within valid bounds.
 *
 * @return `true` if all consistency checks pass, `false` otherwise.
 */
bool cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    check_consistency() const {
  if (!SolGeneralProblemInstanceWithScheduleAndRoutes::check_consistency()) {
    return false;
  }

  if (!this->has_solution()) {
    return true;
  }

  auto const& train_list = this->get_instance()->get_const_train_list();
  for (auto tr_id = 0; tr_id < train_list.size(); tr_id++) {
    const auto& tr_name = train_list.get_train(tr_id).get_name();
    if (!get_const_solution_routes().has_route(tr_name)) {
      return false;
    }
    if (m_train_pos.at(tr_id).size() < 2) {
      // At least two points of information are needed to recover the timing
      return false;
    }

    if (m_train_pos.size() != m_train_speed.size()) {
      return false;
    }

    if (m_train_pos.at(tr_id).size() != m_train_speed.at(tr_id).size()) {
      return false;
    }
    for (const auto& t : m_train_pos.at(tr_id) | std::ranges::views::keys) {
      if (m_train_speed.at(tr_id).count(t) != 1) {
        return false;
      }
    }
    for (const auto& t : m_train_speed.at(tr_id) | std::ranges::views::keys) {
      if (m_train_pos.at(tr_id).count(t) != 1) {
        return false;
      }
    }
  }

  for (const auto& train_pos_vec : m_train_pos) {
    for (const auto& pos : train_pos_vec | std::ranges::views::values) {
      if (pos + EPS < 0) {
        return false;
      }
    }
  }
  for (size_t tr_id = 0; tr_id < m_train_speed.size(); ++tr_id) {
    const auto& train =
        this->get_instance()->get_const_train_list().get_train(tr_id);
    for (const auto& v : m_train_speed.at(tr_id) | std::ranges::views::values) {
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
// NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if)
#if 0
/**
 * @brief Initializes per-edge storage for Variable Speed Section positions.
 */
void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    initialize_vss_vector() {
  m_vss_pos = std::vector<std::vector<double>>(
      this->get_instance()->get_const_network().number_of_edges());
}

/**
 * @brief Adds a VSS position on an edge, optionally also on its reverse edge.
 *
 * @param edge_input The edge specification.
 * @param pos Position on the edge; must be strictly between 0 and the edge length.
 * @param reverse_edge If `true`, also adds the position on the reverse edge.
 *
 * @throws ConsistencyException if `pos` is not strictly between 0 and the edge length.
 */
void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    add_vss_pos(cda_rail::Network::EdgeInput const& edge_input, double pos,
                bool reverse_edge) {
  // Add VSS position on edge. Also on the reverse edge if true.

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

/**
 * @brief Sets the VSS positions for an edge.
 *
 * @param edge_input Identifies the edge.
 * @param pos Vector of positions to store on the edge.
 *
 * @throws ConsistencyException If any position is less than or equal to zero
 *         (within tolerance) or greater than or equal to the edge length
 *         (within tolerance).
 */
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

/**
 * @brief Clears VSS position data for the specified edge.
 *
 * @param edge_input Identifies the edge whose VSS position data should be cleared.
 */
void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    reset_vss_pos(cda_rail::Network::EdgeInput const& edge_input) {
  auto const edge_id =
      this->get_instance()->get_const_network().get_edge_index(edge_input);
  m_vss_pos.at(edge_id).clear();
}

/**
 * @brief Exports the solution including variable speed section positions.
 *
 * Writes solution files and VSS position data for each network edge to
 * vss_pos.json in the export directory.
 */
void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    export_solution(
        const std::filesystem::path& workingDirectory,
        std::string_view const solutionSubdirectory, bool export_instance,
        std::optional<std::string> const& parameter_identifier) const {
  SolGeneralPerformanceOptimizationInstance::export_solution(
      workingDirectory, solutionSubdirectory, export_instance,
      parameter_identifier);

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

  std::filesystem::path const p = get_export_path(
      workingDirectory, solutionSubdirectory, parameter_identifier);
  std::ofstream vss_pos_file(p / "vss_pos.json");
  vss_pos_file << vss_pos_json << '\n';
  vss_pos_file.close();
}

/**
 * @brief Validates that VSS position data is consistent with network edge properties.
 *
 * Checks that parent class consistency holds, VSS positions exist only on breakable edges,
 * and all stored positions fall within their respective edge bounds.
 *
 * @return `true` if all consistency checks pass, `false` otherwise.
 */
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

/**
 * @brief Loads VSS position data from a JSON file into the solution.
 *
 * Delegates to the base class to load the primary solution data, then reads
 * `vss_pos.json` from the export directory. For each edge identified by name,
 * extracts and sorts the corresponding position vector, storing it in
 * `m_vss_pos` indexed by edge ID.
 */
void cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance::
    load_solution(const std::filesystem::path&      workingDirectory,
                  std::string_view const            solutionSubdirectory,
                  std::optional<std::string> const& parameter_identifier) {
  SolGeneralPerformanceOptimizationInstance::load_solution(
      workingDirectory, solutionSubdirectory, parameter_identifier);

  std::filesystem::path const p = this->get_export_path(
      workingDirectory, solutionSubdirectory, parameter_identifier);
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
#endif
