#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_set>
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
  std::vector<std::unordered_set<size_t>> cell_edges;
  std::vector<std::unordered_set<size_t>> cell_vertices;
  std::unordered_map<size_t, size_t>      cell_id_to_index;
  std::vector<std::unordered_set<size_t>> cells_including_vertex(
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
    const size_t edge_index =
        instance.n().get_edge_index("v_" + from_node_id, "v_" + to_node_id);

    // Is cell already known?
    if (cell_id_to_index.find(cell_id) == cell_id_to_index.end()) {
      cell_id_to_index[cell_id] = cell_edges.size();
      cell_edges.emplace_back();
      cell_vertices.emplace_back();
    }
    cell_edges[cell_id_to_index[cell_id]].insert(edge_index);
    if (instance.const_n().has_edge("v_" + to_node_id, "v_" + from_node_id)) {
      cell_edges[cell_id_to_index[cell_id]].insert(
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

  return instance;
};

TEST(RASInstances, CreateToy) {
  const auto instance = create_ras_instance("ras-datasets/toy");
  instance.export_instance("example-networks-gen-po-ras/toy");
}
