#include "probleminstances/VSSGenerationTimetable.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"

#include <numeric>

void cda_rail::instances::VSSGenerationTimetable::export_instance(
    const std::filesystem::path& p) const {
  /**
   * Exports the instance to the given path, i.e.,
   * - the network into the folder "network"
   * - the timetable into the folder "timetable"
   * - the routes into the folder "routes"
   *
   * @param p the path to the folder where the instance should be exported
   */

  if (!cda_rail::is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }
  network.export_network(p / "network");
  timetable.export_timetable(p / "timetable", network);
  routes.export_routes(p / "routes", network);
}

cda_rail::instances::VSSGenerationTimetable::VSSGenerationTimetable(
    const std::filesystem::path& p, bool every_train_must_have_route) {
  /**
   * Creates object and imports an instance from the given path, i.e.,
   * - the network from the folder "network"
   * - the timetable from the folder "timetable"
   * - the routes from the folder "routes"
   *
   * @param p the path to the folder where the instance should be imported from
   */

  this->network   = Network::import_network(p / "network");
  this->timetable = Timetable::import_timetable(p / "timetable", this->network);
  this->routes    = RouteMap::import_routes(p / "routes", this->network);
  if (!this->check_consistency(every_train_must_have_route)) {
    throw exceptions::ConsistencyException();
  }
}

void cda_rail::instances::VSSGenerationTimetable::discretize(
    vss::SeparationFunction sep_func) {
  /**
   * This method discretizes the network. It updates the timetable and the
   * routes accordingly.
   *
   * @param separation_type the type of separation to be used
   */

  const auto new_edges = network.discretize(sep_func);
  timetable.update_after_discretization(new_edges);
  routes.update_after_discretization(new_edges);
}

std::vector<size_t>
cda_rail::instances::VSSGenerationTimetable::trains_in_section(
    const std::vector<size_t>& section) const {
  /**
   * Returns the trains that traverse the given section
   *
   * @param section: the section
   * @return the trains that traverse the given section
   */

  std::vector<size_t> tr_in_sec;
  for (size_t i = 0; i < get_train_list().size(); ++i) {
    auto tr_name        = get_train_list().get_train(i).name;
    auto tr_route       = get_route(tr_name).get_edges();
    bool tr_in_sec_flag = false;
    for (size_t j = 0; j < section.size() && !tr_in_sec_flag; ++j) {
      for (size_t k = 0; k < tr_route.size() && !tr_in_sec_flag; ++k) {
        if (section[j] == tr_route[k]) {
          tr_in_sec.emplace_back(i);
          tr_in_sec_flag = true;
        }
      }
    }
  }

  return tr_in_sec;
}

std::vector<size_t>
cda_rail::instances::VSSGenerationTimetable::trains_at_t(int t) const {
  /**
   * Returns a list of all trains present at time t.
   *
   * @param t the time
   * @return a list of all trains present at time t.
   */
  const auto          tr_number = get_train_list().size();
  std::vector<size_t> trains_to_consider(tr_number);
  std::iota(trains_to_consider.begin(), trains_to_consider.end(), 0);
  return trains_at_t(t, trains_to_consider);
}

std::vector<size_t> cda_rail::instances::VSSGenerationTimetable::trains_at_t(
    int t, const std::vector<size_t>& trains_to_consider) const {
  /**
   * Returns a list of all trains present at time t. But only considers the
   * trains in trains_to_consider.
   *
   * @param t the time
   * @param trains_to_consider the trains to consider
   * @return a list of all trains present at time t. But only considers the
   * trains in trains_to_consider.
   */

  if (t < 0) {
    throw exceptions::InvalidInputException("t must be non-negative.");
  }
  for (auto tr : trains_to_consider) {
    if (!get_train_list().has_train(tr)) {
      throw exceptions::TrainNotExistentException();
    }
  }

  std::vector<size_t> trains;
  for (const auto tr : trains_to_consider) {
    const auto& interval = timetable.time_interval(tr);
    if (interval.first <= t && t < interval.second) {
      trains.push_back(tr);
    }
  }

  return trains;
}

bool cda_rail::instances::VSSGenerationTimetable::has_route_for_every_train()
    const {
  /**
   * Checks if every train has a route.
   *
   * @return true if every train has a route, false otherwise
   */

  return std::all_of(get_train_list().begin(), get_train_list().end(),
                     [this](const auto& tr) {
                       return has_route(tr.name) && !get_route(tr.name).empty();
                     });
}

std::vector<size_t>
cda_rail::instances::VSSGenerationTimetable::edges_used_by_train(
    const std::string& train_name, bool fixed_routes) const {
  /**
   * Returns edges potentially used by a specific train.
   *
   * @param train_name the name of the train
   * @param fixed_routes specifies if the routes are fixed, if not returns all
   * edges
   *
   * @return edges potentially used by a specific train
   */

  if (!fixed_routes) {
    // return vector with values 0, 1, ..., num_edges-1
    std::vector<size_t> return_edges(network.number_of_edges());
    std::iota(return_edges.begin(), return_edges.end(), 0);
    return return_edges;
  }
  return get_route(train_name).get_edges();
}

std::vector<size_t> cda_rail::instances::VSSGenerationTimetable::trains_on_edge(
    size_t edge_id, bool fixed_routes) const {
  /**
   * Returns all trains that are present on a specific edge at any time.
   *
   * @param edge_id the id of the edge
   * @param fixed_routes specifies if the routes are fixed, if not returns all
   * trains
   *
   * @return all trains that are present on a specific edge at any time
   */
  std::vector<size_t> trains_to_consider(get_train_list().size());
  std::iota(trains_to_consider.begin(), trains_to_consider.end(), 0);
  return trains_on_edge(edge_id, fixed_routes, trains_to_consider);
}

std::vector<size_t> cda_rail::instances::VSSGenerationTimetable::trains_on_edge(
    size_t edge_id, bool fixed_routes,
    const std::vector<size_t>& trains_to_consider) const {
  /**
   * Returns all trains that are present on a specific edge, but only consider a
   * subset of trains.
   *
   * @param edge_id the id of the edge
   * @param fixed_routes specifies if the routes are fixed, if not returns all
   * trains
   * @param trains_to_consider the trains to consider
   *
   * @return all trains that are present on a specific edge, but only consider a
   * subset of trains
   */

  if (!network.has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  if (!fixed_routes) {
    return trains_to_consider;
  }
  std::vector<size_t> return_trains;
  for (const auto tr : trains_to_consider) {
    const auto& tr_route = this->get_route(get_train_list().get_train(tr).name);
    if (tr_route.contains_edge(edge_id)) {
      return_trains.push_back(tr);
    }
  }
  return return_trains;
}
