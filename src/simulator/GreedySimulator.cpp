#include "simulator/GreedySimulator.hpp"

#include <cstddef>

bool cda_rail::simulator::GreedySimulator::check_consistency() const {
  if (!instance->check_consistency(false)) {
    return false;
  }

  // All vectors are of correct size
  if (train_edges.size() != instance->get_timetable().get_train_list().size()) {
    return false;
  }
  if (ttd_orders.size() != ttd_sections.size()) {
    return false;
  }
  if (entry_orders.size() != instance->const_n().number_of_vertices()) {
    return false;
  }

  // All edges are valid
  for (const auto& edges : train_edges) {
    for (const auto& edge : edges) {
      if (!instance->const_n().has_edge(edge)) {
        return false;
      }
    }
  }

  // All edges are valid successors of each other
  for (const auto& edges : train_edges) {
    const auto& s = edges.size();
    for (size_t i = 0; s > 1 && i < s - 1; ++i) {
      if (!instance->const_n().is_valid_successor(edges[i], edges[i + 1])) {
        return false;
      }
    }
  }

  // If not empty, the first edge of each train must be an entry edge
  for (size_t train_id = 0; train_id < train_edges.size(); ++train_id) {
    if (!train_edges[train_id].empty()) {
      const auto& first_edge_id = train_edges[train_id].front();
      const auto& first_edge    = instance->const_n().get_edge(first_edge_id);
      if (first_edge.source !=
          instance->get_timetable().get_schedule(train_id).get_entry()) {
        return false;
      }
    }
  }

  // ttd_orders only contains valid train indices
  for (const auto& orders : ttd_orders) {
    for (const auto& train_id : orders) {
      if (!instance->get_timetable().get_train_list().has_train(train_id)) {
        return false;
      }
    }
  }

  // entry_orders only contains valid train indices
  for (const auto& orders : entry_orders) {
    for (const auto& train_id : orders) {
      if (!instance->get_timetable().get_train_list().has_train(train_id)) {
        return false;
      }
    }
  }

  // entry orders only contain trains that are entering the network through the
  // respective vertex
  for (size_t vertex_id = 0; vertex_id < entry_orders.size(); ++vertex_id) {
    for (const auto& train_id : entry_orders[vertex_id]) {
      if (vertex_id !=
          instance->get_timetable().get_schedule(train_id).get_entry()) {
        return false;
      }
    }
  }

  return true;
}
