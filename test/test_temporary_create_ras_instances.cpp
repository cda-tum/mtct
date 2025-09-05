#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "gtest/gtest.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

cda_rail::instances::GeneralPerformanceOptimizationInstance
create_ras_instance(const std::string& path) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;

  // Convert to path
  const auto path_to_instance = std::filesystem::path(path);
  if (!std::filesystem::exists(path_to_instance)) {
    throw std::exception("Path does not exist");
  }

  // Load Input_Node.csv, which contains a list of node_ids (except the first
  // line)
  const auto path_to_input_node = path_to_instance / "Input_Node.csv";
  if (!std::filesystem::exists(path_to_input_node)) {
    throw std::exception("Input_Node.csv does not exist");
  }
  // Read file line by line without the first line
  std::ifstream input_node_file(path_to_input_node);
  std::string   line;
  while (std::getline(input_node_file, line)) {
    if (line == "node_id") {
      continue;
    }
    instance.n().add_vertex("v_" + line, cda_rail::VertexType::TTD);
  }
  input_node_file.close();

  // From Input_Link.csv extract edges
  // from_node_id,to_node_id,length_in_mile,speed_limit_in_mph_FT,speed_limit_in_mph_TF,link_capacity_in_train_per_time_interval,dwelling_allowable_flag,bidirectional_flag
  const auto path_to_input_link = path_to_instance / "Input_Link.csv";
  if (!std::filesystem::exists(path_to_input_link)) {
    throw std::exception("Input_Link.csv does not exist");
  }
  // Read file line by line without the first line
  std::ifstream input_link_file(path_to_input_link);
  while (std::getline(input_link_file, line)) {
    if (line == "from_node_id,to_node_id,length_in_mile,speed_limit_in_mph_FT,"
                "speed_limit_in_mph_TF,link_capacity_in_train_per_time_"
                "interval,dwelling_allowable_flag,bidirectional_flag") {
      continue;
    }
    std::istringstream iss(line);
    std::string        from_node_id, to_node_id, length_in_mile_str,
        speed_limit_in_mph_FT_str, speed_limit_in_mph_TF_str,
        link_capacity_in_train_per_time_interval_str,
        dwelling_allowable_flag_str, bidirectional_flag_str;

    std::getline(iss, from_node_id, ',');
    std::getline(iss, to_node_id, ',');
    std::getline(iss, length_in_mile_str, ',');
    std::getline(iss, speed_limit_in_mph_FT_str, ',');
    std::getline(iss, speed_limit_in_mph_TF_str, ',');
    std::getline(iss, link_capacity_in_train_per_time_interval_str, ',');
    std::getline(iss, dwelling_allowable_flag_str, ',');
    std::getline(iss, bidirectional_flag_str, ',');

    double length_in_meter = std::stod(length_in_mile_str) * 1609.344;
    double speed_limit_in_mps_FT =
        std::stod(speed_limit_in_mph_FT_str) * 0.44704;
    double speed_limit_in_mps_TF =
        std::stod(speed_limit_in_mph_TF_str) * 0.44704;
    bool bidirectional_flag = (bidirectional_flag_str == "1");

    instance.n().add_edge("v_" + from_node_id, "v_" + to_node_id,
                          length_in_meter, speed_limit_in_mps_FT);
    if (bidirectional_flag) {
      instance.n().add_edge("v_" + to_node_id, "v_" + from_node_id,
                            length_in_meter, speed_limit_in_mps_TF);
    }
  }

  // Extract cells from Input_Cell.csv
  // cell_id,from_node_id,to_node_id
  const auto path_to_input_cell = path_to_instance / "Input_Cell.csv";
  if (!std::filesystem::exists(path_to_input_cell)) {
    throw std::exception("Input_Cell.csv does not exist");
  }
  std::vector<std::vector<size_t>>   cell_edges;
  std::vector<std::set<size_t>>      cell_vertices;
  std::unordered_map<size_t, size_t> cell_id_to_index;
  std::vector<std::set<size_t>>      cells_including_vertex(
      instance.const_n().number_of_vertices());
  std::ifstream input_cell_file(path_to_input_cell);
  while (std::getline(input_cell_file, line)) {
    if (line == "cell_id,from_node_id,to_node_id") {
      continue;
    }
    std::istringstream iss(line);
    std::string        cell_id_str;
    std::string        from_node_id;
    std::string        to_node_id;

    std::getline(iss, cell_id_str, ',');
    std::getline(iss, from_node_id, ',');
    std::getline(iss, to_node_id, ',');

    const size_t cell_id = std::stoul(cell_id_str);
    const size_t source_vertex_index =
        instance.n().get_vertex_index("v_" + from_node_id);
    const size_t target_vertex_index =
        instance.n().get_vertex_index("v_" + to_node_id);

    // Is cell already known?
    if (cell_id_to_index.find(cell_id) == cell_id_to_index.end()) {
      cell_id_to_index[cell_id] = cell_edges.size();
      cell_edges.emplace_back();
      cell_vertices.emplace_back();
    }
    if (instance.const_n().has_edge("v_" + from_node_id, "v_" + to_node_id)) {
      cell_edges[cell_id_to_index[cell_id]].push_back(
          instance.const_n().get_edge_index(source_vertex_index,
                                            target_vertex_index));
    }
    if (instance.const_n().has_edge("v_" + to_node_id, "v_" + from_node_id)) {
      cell_edges[cell_id_to_index[cell_id]].push_back(
          instance.const_n().get_edge_index(target_vertex_index,
                                            source_vertex_index));
    }
    cell_vertices[cell_id_to_index[cell_id]].insert(source_vertex_index);
    cell_vertices[cell_id_to_index[cell_id]].insert(target_vertex_index);
    cells_including_vertex[source_vertex_index].insert(
        cell_id_to_index[cell_id]);
    cells_including_vertex[target_vertex_index].insert(
        cell_id_to_index[cell_id]);
  }

  // Vertices that are only part of one cell and have more than one neighbor are
  // no border vertices
  for (size_t vertex_index = 0;
       vertex_index < instance.const_n().number_of_vertices(); ++vertex_index) {
    if (cells_including_vertex[vertex_index].size() == 1 &&
        instance.const_n().neighbors(vertex_index).size() > 1) {
      instance.n().change_vertex_type(vertex_index,
                                      cda_rail::VertexType::NoBorder);
      for (const auto e : instance.const_n().neighboring_edges(vertex_index)) {
        instance.n().set_edge_unbreakable(e);
      }
    }
  }

  // Vertices with two neighbors are straight tracks
  for (size_t vertex_index = 0;
       vertex_index < instance.const_n().number_of_vertices(); ++vertex_index) {
    if (const auto neighbors_tmp = instance.const_n().neighbors(vertex_index);
        neighbors_tmp.size() == 2) {
      // Hence, no travel restrictions
      if (instance.const_n().has_edge(neighbors_tmp[0], vertex_index) &&
          instance.const_n().has_edge(vertex_index, neighbors_tmp[1])) {
        instance.n().add_successor({neighbors_tmp[0], vertex_index},
                                   {vertex_index, neighbors_tmp[1]});
      }
      if (instance.const_n().has_edge(neighbors_tmp[1], vertex_index) &&
          instance.const_n().has_edge(vertex_index, neighbors_tmp[0])) {
        instance.n().add_successor({neighbors_tmp[1], vertex_index},
                                   {vertex_index, neighbors_tmp[0]});
      }
    }
  }

  // For others, extract from Input_Block_Section.csv
  // block_section_id,cell_sequence_number,cell_id
  const auto path_to_input_block_section =
      path_to_instance / "Input_Block_Section.csv";
  std::vector<std::vector<size_t>> block_sections;
  if (!std::filesystem::exists(path_to_input_block_section)) {
    throw std::exception("Input_Block_Section.csv does not exist");
  }
  std::ifstream input_block_section_file(path_to_input_block_section);
  int           last_block_section_id = -1;
  size_t        last_sequence_id      = 0;
  while (std::getline(input_block_section_file, line)) {
    if (line == "block_section_id,cell_sequence_number,cell_id") {
      continue;
    }
    std::istringstream iss(line);
    std::string        block_section_id_str;
    std::string        cell_sequence_number_str;
    std::string        cell_id_str;

    std::getline(iss, block_section_id_str, ',');
    std::getline(iss, cell_sequence_number_str, ',');
    std::getline(iss, cell_id_str, ',');

    const int    block_section_id     = std::stoi(block_section_id_str);
    const size_t cell_sequence_number = std::stoul(cell_sequence_number_str);
    const size_t cell_id              = std::stoul(cell_id_str);

    if (block_section_id != last_block_section_id) {
      block_sections.emplace_back();
      last_sequence_id = cell_sequence_number - 1;
    }

    assert(cell_sequence_number == last_sequence_id + 1);
    block_sections.back().push_back(cell_id_to_index[cell_id]);
    last_sequence_id      = cell_sequence_number;
    last_block_section_id = block_section_id;
  }

  // Deduce successors from block sections
  for (const auto& block_section : block_sections) {
    if (block_section.size() < 2) {
      // Some vertices are obvious because the edges are non-directional
      const auto& cell = cell_vertices[block_section[0]];
      for (const auto& vertex_index : cell) {
        std::set<size_t> neighbors_tmp;
        for (const auto n : instance.const_n().neighbors(vertex_index)) {
          if (cell.contains(n)) {
            neighbors_tmp.insert(n);
          }
        }
        if (neighbors_tmp.size() >= 3) {
          const auto in_edges  = instance.const_n().in_edges(vertex_index);
          const auto out_edges = instance.const_n().out_edges(vertex_index);
          bool       none_have_reverse = true;
          for (const auto& e : in_edges) {
            if (instance.const_n().get_reverse_edge_index(e).has_value()) {
              none_have_reverse = false;
              break;
            }
          }
          for (const auto& e : out_edges) {
            if (instance.const_n().get_reverse_edge_index(e).has_value()) {
              none_have_reverse = false;
              break;
            }
          }
          if (none_have_reverse) {
            // All edges are one-way, hence all possible successors are
            // allowed
            for (const auto& e_in : in_edges) {
              const auto e_in_tmp_obj = instance.const_n().get_edge(e_in);
              assert(e_in_tmp_obj.target == vertex_index);
              assert(cell.contains(e_in_tmp_obj.source));
              for (const auto& e_out : out_edges) {
                const auto e_out_tmp_obj = instance.const_n().get_edge(e_out);
                assert(e_out_tmp_obj.source == vertex_index);
                assert(cell.contains(e_out_tmp_obj.target));
                instance.n().add_successor(e_in, e_out);
              }
            }
          }
        }
      }
      continue;
    }
    assert(block_section.size() >= 2);

    // First and last cell
    const auto& first_cell  = cell_vertices[block_section[0]];
    const auto& second_cell = cell_vertices[block_section[1]];
    const auto& last_cell =
        cell_vertices[block_section[block_section.size() - 1]];
    const auto& second_to_last_cell =
        cell_vertices[block_section[block_section.size() - 2]];

    // Intersection first
    std::set<size_t> intersection_first;
    std::set_intersection(
        first_cell.begin(), first_cell.end(), second_cell.begin(),
        second_cell.end(),
        std::inserter(intersection_first, intersection_first.begin()));
    // Intersection last
    std::set<size_t> intersection_last;
    std::set_intersection(
        second_to_last_cell.begin(), second_to_last_cell.end(),
        last_cell.begin(), last_cell.end(),
        std::inserter(intersection_last, intersection_last.begin()));

    // First section
    if (first_cell.size() >= 3) {
      const auto& relevant_edges_first = cell_edges[block_section[0]];

      // Get all entry edges, i.e.,
      // for every vertex in cell
      // with one neighbor within the cell
      // the edge connecting the border vertex with the neighbor within the
      // cell.
      std::vector<size_t> entering_edges_prev_cell;
      for (const auto& v : first_cell) {
        if (v == *intersection_first.begin()) {
          continue;
        }
        const auto neighbors        = instance.const_n().neighbors(v);
        size_t     neighbor_in_cell = 0;
        size_t     rel_n            = 0;
        for (const auto& n : neighbors) {
          if (first_cell.contains(n)) {
            rel_n = n;
            neighbor_in_cell++;
          }
        }
        if (neighbor_in_cell == 1 && instance.const_n().has_edge(v, rel_n)) {
          entering_edges_prev_cell.emplace_back(
              instance.const_n().get_edge_index(v, rel_n));
        }
      }

      // Path from border vertices to first intersection vertex
      assert(!entering_edges_prev_cell.empty());
      for (const auto& entering_e : entering_edges_prev_cell) {
        const auto [relevant_path_length, relevant_path] =
            instance.const_n().shortest_path_using_edges(
                entering_e, *intersection_first.begin(), false,
                relevant_edges_first);
        assert(relevant_path_length.has_value());
        assert(!relevant_path.empty());
        for (size_t j = 0; j < relevant_path.size() - 1; ++j) {
          instance.n().add_successor(relevant_path[j], relevant_path[j + 1]);
          const auto e_j_reverse =
              instance.const_n().get_reverse_edge_index(relevant_path[j]);
          const auto e_j_plus_1_reverse =
              instance.const_n().get_reverse_edge_index(relevant_path[j + 1]);
          if (e_j_reverse.has_value() && e_j_plus_1_reverse.has_value()) {
            instance.n().add_successor(e_j_plus_1_reverse.value(),
                                       e_j_reverse.value());
          }
        }
      }
    }

    // Last section
    if (last_cell.size() >= 3) {
      const auto& relevant_edges_last =
          cell_edges[block_section[block_section.size() - 1]];
      std::optional<size_t> first_edge;
      for (const auto e :
           instance.const_n().out_edges(*intersection_last.begin())) {
        if (std::ranges::contains(relevant_edges_last, e)) {
          first_edge = e;
          break;
        }
      }
      assert(first_edge.has_value());
      std::vector<size_t> exiting_vertices;
      for (const auto& v : last_cell) {
        if (v == *intersection_last.begin()) {
          continue;
        }
        const auto neighbors        = instance.const_n().neighbors(v);
        size_t     neighbor_in_cell = 0;
        for (const auto& n : neighbors) {
          if (last_cell.contains(n)) {
            neighbor_in_cell++;
          }
        }
        if (neighbor_in_cell == 1) {
          exiting_vertices.push_back(v);
        }
      }

      // Path through last cell
      assert(!exiting_vertices.empty());
      for (const auto& exiting_v : exiting_vertices) {
        const auto [relevant_path_length, relevant_path] =
            instance.const_n().shortest_path_using_edges(
                first_edge.value(), exiting_v, false, relevant_edges_last);
        if (!relevant_path_length.has_value()) {
          continue;
        }
        assert(relevant_path_length.has_value());
        assert(!relevant_path.empty());
        for (size_t j = 0; j < relevant_path.size() - 1; ++j) {
          instance.n().add_successor(relevant_path[j], relevant_path[j + 1]);
          const auto e_j_reverse =
              instance.const_n().get_reverse_edge_index(relevant_path[j]);
          const auto e_j_plus_1_reverse =
              instance.const_n().get_reverse_edge_index(relevant_path[j + 1]);
          if (e_j_reverse.has_value() && e_j_plus_1_reverse.has_value()) {
            instance.n().add_successor(e_j_plus_1_reverse.value(),
                                       e_j_reverse.value());
          }
        }
      }
    }

    // Middle cells
    for (size_t i = 1; i < block_section.size() - 1; ++i) {
      const auto& previous_cell = cell_vertices[block_section[i - 1]];
      const auto& current_cell  = cell_vertices[block_section[i]];
      const auto& next_cell     = cell_vertices[block_section[i + 1]];

      // Intersection of previous and current cell
      std::set<size_t> intersection_prev;
      std::set_intersection(
          previous_cell.begin(), previous_cell.end(), current_cell.begin(),
          current_cell.end(),
          std::inserter(intersection_prev, intersection_prev.begin()));
      // Intersection of current and next cell
      std::set<size_t> intersection_next;
      std::set_intersection(
          current_cell.begin(), current_cell.end(), next_cell.begin(),
          next_cell.end(),
          std::inserter(intersection_next, intersection_next.begin()));

      assert(intersection_prev.size() == 1);
      assert(intersection_next.size() == 1);
      assert(instance.const_n().neighbors(*intersection_prev.begin()).size() ==
             2);
      assert(instance.const_n().neighbors(*intersection_next.begin()).size() ==
             2);

      // Path through middle section
      const auto out_edges =
          instance.const_n().out_edges(*intersection_prev.begin());
      const auto& relevant_edges = cell_edges[block_section[i]];
      for (const auto& edge : out_edges) {
        if (std::find(relevant_edges.begin(), relevant_edges.end(), edge) !=
            relevant_edges.end()) {
          const auto [relevant_path_length, relevant_path] =
              instance.const_n().shortest_path_using_edges(
                  edge, *intersection_next.begin(), false, relevant_edges);
          assert(relevant_path_length.has_value());
          assert(!relevant_path.empty());
          for (size_t j = 0; j < relevant_path.size() - 1; ++j) {
            instance.n().add_successor(relevant_path[j], relevant_path[j + 1]);
            const auto e_j_reverse =
                instance.const_n().get_reverse_edge_index(relevant_path[j]);
            const auto e_j_plus_1_reverse =
                instance.const_n().get_reverse_edge_index(relevant_path[j + 1]);
            if (e_j_reverse.has_value() && e_j_plus_1_reverse.has_value()) {
              instance.n().add_successor(e_j_plus_1_reverse.value(),
                                         e_j_reverse.value());
            }
          }
        }
      }
    }
  }

  // Some vertices are missing in data
  for (size_t vertex_index = 0;
       vertex_index < instance.const_n().number_of_vertices(); ++vertex_index) {
    if (instance.const_n().neighbors(vertex_index).size() >= 3) {
      const auto in_edges          = instance.const_n().in_edges(vertex_index);
      const auto out_edges         = instance.const_n().out_edges(vertex_index);
      bool       none_have_reverse = true;
      bool       none_have_successors = true;
      for (const auto& e : in_edges) {
        if (!instance.const_n().get_successors(e).empty()) {
          none_have_successors = false;
          break;
        }
        if (instance.const_n().get_reverse_edge_index(e).has_value()) {
          none_have_reverse = false;
          break;
        }
      }
      for (const auto& e : out_edges) {
        if (!instance.const_n().get_predecessors(e).empty()) {
          none_have_successors = false;
          break;
        }
        if (instance.const_n().get_reverse_edge_index(e).has_value()) {
          none_have_reverse = false;
          break;
        }
      }
      if (none_have_reverse && none_have_successors) {
        // All edges are one-way, hence all possible successors are
        // allowed
        for (const auto& e_in : in_edges) {
          for (const auto& e_out : out_edges) {
            instance.n().add_successor(e_in, e_out);
          }
        }
      }
    }
  }

  // Extract stations from InputM_Stations.csv
  // station_name,cell_id
  const auto path_to_input_station = path_to_instance / "InputM_Stations.csv";
  if (!std::filesystem::exists(path_to_input_station)) {
    throw std::exception("InputM_Station.csv does not exist");
  }
  std::ifstream input_station_file(path_to_input_station);
  while (std::getline(input_station_file, line)) {
    if (line == "station_name,cell_id") {
      continue;
    }
    std::istringstream iss(line);
    std::string        station_name;
    std::string        cell_id_str;

    std::getline(iss, station_name, ',');
    std::getline(iss, cell_id_str, ',');

    const size_t cell_id    = std::stoul(cell_id_str);
    const size_t cell_index = cell_id_to_index[cell_id];
    const auto&  cell       = cell_edges[cell_index];
    if (!instance.get_station_list().has_station(station_name)) {
      instance.add_station(station_name);
    }
    for (const auto& e : cell) {
      instance.add_track_to_station(station_name, e);
    }
  }

  double max_speed = 0;
  for (size_t i = 0; i < instance.const_n().number_of_edges(); ++i) {
    max_speed = std::max(max_speed, instance.const_n().get_edge(i).max_speed);
  }

  double min_station_length = std::numeric_limits<double>::max();
  for (const auto& [station_name, station] : instance.get_station_list()) {
    for (const auto& track_id : station.tracks) {
      min_station_length = std::min(
          min_station_length, instance.const_n().get_edge(track_id).length);
    }
  }

  // Extract trains from Input_Train_Info.csv
  // train_id,origin_node_id,destination_node_id,speed
  // multiplier,earliest_departure_time,latest_departure_time
  const auto path_to_input_train_info =
      path_to_instance / "Input_Train_Info.csv";
  if (!std::filesystem::exists(path_to_input_train_info)) {
    throw std::exception("Input_Train_Info.csv does not exist");
  }
  std::ifstream input_train_info_file(path_to_input_train_info);
  while (std::getline(input_train_info_file, line)) {
    if (line == "train_id,origin_node_id,destination_node_id,speed "
                "multiplier,earliest_departure_time,latest_departure_time") {
      continue;
    }
    std::istringstream iss(line);
    std::string        train_id;
    std::string        origin_node_id;
    std::string        destination_node_id;
    std::string        speed_multiplier_str;
    std::string        earliest_departure_time_str;
    std::string        latest_departure_time_str;

    std::getline(iss, train_id, ',');
    std::getline(iss, origin_node_id, ',');
    std::getline(iss, destination_node_id, ',');
    std::getline(iss, speed_multiplier_str, ',');
    std::getline(iss, earliest_departure_time_str, ',');
    std::getline(iss, latest_departure_time_str, ',');

    const double speed_multiplier     = std::stod(speed_multiplier_str);
    const int earliest_departure_time = std::stoi(earliest_departure_time_str);
    const int latest_departure_time   = std::stoi(latest_departure_time_str);

    const size_t origin_vertex_index =
        instance.n().get_vertex_index("v_" + origin_node_id);
    const size_t destination_vertex_index =
        instance.n().get_vertex_index("v_" + destination_node_id);

    const auto in_edges = instance.const_n().out_edges(origin_vertex_index);
    double     initial_speed = std::numeric_limits<double>::max();
    for (const auto& e : in_edges) {
      initial_speed =
          std::min(initial_speed,
                   speed_multiplier * instance.const_n().get_edge(e).max_speed);
    }

    const auto out_edges =
        instance.const_n().in_edges(destination_vertex_index);
    double target_speed = std::numeric_limits<double>::max();
    for (const auto& e : out_edges) {
      target_speed =
          std::min(target_speed,
                   speed_multiplier * instance.const_n().get_edge(e).max_speed);
    }

    const double acceleration = 1;
    const double deceleration = 0.9;

    for (const auto& [station_name_tmp, station_tmp] :
         instance.get_station_list()) {
      const auto p_len_tmp = instance.const_n().shortest_path_between_sets(
          in_edges, station_tmp.tracks, true, true);
      if (p_len_tmp.has_value()) {
        initial_speed = std::min(
            initial_speed, std::min(speed_multiplier, 0.75) *
                               std::sqrt(2 * deceleration * p_len_tmp.value()));
      }

      const auto p_len_tmp_2 = instance.const_n().shortest_path_between_sets(
          station_tmp.tracks, out_edges, true, true);
      if (p_len_tmp_2.has_value()) {
        target_speed =
            std::min(target_speed,
                     std::min(speed_multiplier, 0.75) *
                         std::sqrt(2 * acceleration * p_len_tmp_2.value()));
      }
    }

    instance.add_train(
        "tr_" + train_id, std::min(400.0, min_station_length),
        speed_multiplier * max_speed, acceleration, deceleration,
        {earliest_departure_time, latest_departure_time}, initial_speed,
        origin_vertex_index,
        {earliest_departure_time, latest_departure_time + 6 * 60 * 60},
        target_speed, destination_vertex_index);
  }

  // Extract stops from Input_Train_Required_Stop.csv
  // train_id,station_name,require_stop,minimum_dwelling_time_in_minute
  const auto path_to_input_train_required_stop =
      path_to_instance / "Input_Train_Required_Stop.csv";
  if (!std::filesystem::exists(path_to_input_train_required_stop)) {
    throw std::exception("Input_Train_Required_Stop.csv does not exist");
  }
  std::ifstream input_train_required_stop_file(
      path_to_input_train_required_stop);
  while (std::getline(input_train_required_stop_file, line)) {
    if (line ==
        "train_id,station_name,require_stop,minimum_dwelling_time_in_minute") {
      continue;
    }
    std::istringstream iss(line);
    std::string        train_id;
    std::string        station_name;
    std::string        require_stop_str;
    std::string        minimum_dwelling_time_in_minute_str;

    std::getline(iss, train_id, ',');
    std::getline(iss, station_name, ',');
    std::getline(iss, require_stop_str, ',');
    std::getline(iss, minimum_dwelling_time_in_minute_str, ',');

    const bool require_stop = require_stop_str == "1";
    const int  minimum_dwelling_time_in_minute =
        std::stoi(minimum_dwelling_time_in_minute_str);
    const auto tr_schedule = instance.get_schedule("tr_" + train_id);

    if (require_stop) {
      instance.add_stop(
          "tr_" + train_id, station_name,
          std::pair<int, int>({tr_schedule.get_t_0_range().first,
                               tr_schedule.get_t_n_range().second}),
          std::pair<int, int>({tr_schedule.get_t_0_range().first,
                               tr_schedule.get_t_n_range().second}),
          minimum_dwelling_time_in_minute * 60);
    }
  }

  return instance;
};

TEST(RASInstances, CreateRAS) {
  const auto instance = create_ras_instance("ras-datasets/toy");
  instance.export_instance("example-networks-gen-po-ras/toy");

  const auto instance_practical = create_ras_instance("ras-datasets/practical");
  instance_practical.export_instance("example-networks-gen-po-ras/practical");
}
