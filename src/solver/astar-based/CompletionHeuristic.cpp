#include "solver/astar-based/CompletionHeuristic.hpp"

#include "Definitions.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"
#include "simulator/GeneralSimulator.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <map>
#include <numeric>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

namespace {
// Quickest path by which train tr continues from the end of its route, or from
// its entry vertex if the route is empty, to one of the targets. It is empty
// if no target can be reached by at least one more edge.
cda_rail::index_vector quickest_extension(
    const cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes&
           instance,
    size_t tr, const cda_rail::index_vector& route,
    const cda_rail::index_set& targets, bool targets_are_vertices) {
  const auto& network = instance.get_const_network();
  const auto  max_v =
      instance.get_const_train_list().get_train(tr).get_max_speed();
  const bool from_entry = route.empty();
  const auto sources =
      from_entry ? network.out_edges(
                       instance.get_const_schedule(tr).get_entry_vertex())
                 : cda_rail::index_set{route.back()};
  const auto [time, path] =
      targets_are_vertices
          ? network.shortest_path_between_edge_and_vertex_set(
                sources, targets, true, {}, from_entry, true, max_v)
          : network.shortest_path_between_edge_sets(sources, targets, true, {},
                                                    from_entry, true, max_v);
  if (!time.has_value()) {
    return {};
  }
  if (from_entry) {
    return path;
  }
  // The path starts with the last edge of the route
  return {std::next(path.begin()), path.end()};
}

// The edge of the route on which a train stopping at pos stands, i.e., the
// first one ending there or later
size_t stop_edge(const cda_rail::Network&      network,
                 const cda_rail::index_vector& route, double pos) {
  double end = 0.0;
  for (size_t i = 0; i + 1 < route.size(); ++i) {
    end += network.get_edge(route.at(i)).length;
    // The stop position may have been summed up in another order
    if (end >= pos - cda_rail::GRB_EPS) {
      return route.at(i);
    }
  }
  return route.back();
}
} // namespace

std::optional<cda_rail::simulator::SimulatorState>
cda_rail::solver::astar_based::complete_routes(
    const instances::GeneralProblemInstanceWithScheduleAndRoutes& instance,
    simulator::SimulatorState                                     state) {
  const auto& network = instance.get_const_network();

  cda_rail::index_vector trains(instance.get_const_train_list().size());
  std::iota(trains.begin(), trains.end(), 0);
  std::ranges::stable_sort(trains, {}, [&instance](size_t tr) {
    return instance.get_const_schedule(tr).get_entry_time();
  });

  // When each platform was last used, counting the stops of the trains in the
  // order of their entry times
  std::map<size_t, size_t> last_use;
  size_t                   uses = 0;
  for (const auto tr : trains) {
    const auto& schedule       = instance.get_const_schedule(tr);
    const auto& stops          = schedule.get_stops();
    const auto  exit_vertex    = schedule.get_exit_vertex();
    auto&       route          = state.train_edges.at(tr);
    auto&       stop_positions = state.stop_positions.at(tr);

    // The stops of the state are on the platforms it chose
    for (const auto stop_position : stop_positions) {
      last_use[stop_edge(network, route, stop_position)] = ++uses;
    }

    const auto can_continue = [&](const cda_rail::index_vector& edges,
                                  size_t                        stop) {
      if (stop + 1 < stops.size()) {
        return !quickest_extension(instance, tr, edges,
                                   stops.at(stop + 1).get_station().tracks,
                                   false)
                    .empty();
      }
      return network.get_edge(edges.back()).target == exit_vertex ||
             !quickest_extension(instance, tr, edges, {exit_vertex}, true)
                  .empty();
    };

    for (size_t stop = stop_positions.size(); stop < stops.size(); ++stop) {
      const auto& station_tracks = stops.at(stop).get_station().tracks;
      cda_rail::index_vector tracks(station_tracks.begin(),
                                    station_tracks.end());
      std::ranges::sort(tracks);
      cda_rail::index_vector              platforms;
      std::vector<cda_rail::index_vector> extensions;
      for (const auto track : tracks) {
        auto extension =
            quickest_extension(instance, tr, route, {track}, false);
        if (extension.empty()) {
          continue;
        }
        auto extended_route = route;
        extended_route.insert(extended_route.end(), extension.begin(),
                              extension.end());
        if (instance.is_route_end_valid_stop_pos(tr, extended_route, stop) &&
            can_continue(extended_route, stop)) {
          platforms.emplace_back(track);
          extensions.emplace_back(std::move(extension));
        }
      }
      if (platforms.empty()) {
        return std::nullopt;
      }
      // The platform used least recently, so that trains take turns
      const auto platform =
          std::ranges::min(std::views::iota(size_t{0}, platforms.size()), {},
                           [&last_use, &platforms](size_t i) {
                             const auto it = last_use.find(platforms.at(i));
                             return it == last_use.end() ? 0 : it->second;
                           });
      last_use[platforms.at(platform)] = ++uses;
      const auto& extension            = extensions.at(platform);
      route.insert(route.end(), extension.begin(), extension.end());
      stop_positions.emplace_back(network.length_of_path(route));
    }

    if (route.empty() || network.get_edge(route.back()).target != exit_vertex) {
      const auto extension =
          quickest_extension(instance, tr, route, {exit_vertex}, true);
      if (extension.empty()) {
        return std::nullopt;
      }
      route.insert(route.end(), extension.begin(), extension.end());
    }

    if (cda_rail::index_set(route.begin(), route.end()).size() !=
        route.size()) {
      // Positions on a route that uses an edge twice are ambiguous
      return std::nullopt;
    }
  }
  return state;
}

std::optional<cda_rail::simulator::SimulatorState>
cda_rail::solver::astar_based::complete_state(
    simulator::GeneralSimulator& simulator, simulator::SimulatorState state,
    double dt, bool late_entry_possible, bool limit_speed_by_leaving_edges) {
  auto routed_state =
      complete_routes(*simulator.get_instance(), std::move(state));
  if (!routed_state.has_value()) {
    return std::nullopt;
  }
  simulator.set_simulator_state(std::move(routed_state.value()));
  return simulator.first_come_first_served_state(dt, late_entry_possible,
                                                 limit_speed_by_leaving_edges);
}
