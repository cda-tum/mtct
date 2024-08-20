#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "gtest/gtest.h"
#include <filesystem>

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

  return instance;
};

TEST(RASInstances, CreateToy) {
  const auto instance = create_ras_instance("ras-datasets/toy");
  instance.export_instance("example-networks-gen-po-ras/toy");
}
