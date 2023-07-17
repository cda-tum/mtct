#include "datastructure/Train.hpp"

#include "Definitions.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <fstream>
#include <string>

using json = nlohmann::json;

size_t cda_rail::TrainList::add_train(const std::string& name, int length,
                                      double max_speed, double acceleration,
                                      double deceleration) {
  if (has_train(name)) {
    throw std::invalid_argument("Train already exists.");
  }
  trains.emplace_back(name, length, max_speed, acceleration, deceleration);
  train_name_to_index[name] = trains.size() - 1;
  return train_name_to_index[name];
}

size_t cda_rail::TrainList::get_train_index(const std::string& name) const {
  if (!has_train(name)) {
    throw std::invalid_argument("Train does not exist.");
  }
  return train_name_to_index.at(name);
}

const cda_rail::Train& cda_rail::TrainList::get_train(size_t index) const {
  if (!has_train(index)) {
    throw std::invalid_argument("Train does not exist.");
  }
  return trains.at(index);
}

void cda_rail::TrainList::export_trains(const std::filesystem::path& p) const {
  if (!cda_rail::is_directory_and_create(p)) {
    throw std::runtime_error("Could not create directory " + p.string());
  }

  json j;
  for (const auto& train : trains) {
    j[train.name] = {{"length", train.length},
                     {"max_speed", train.max_speed},
                     {"acceleration", train.acceleration},
                     {"deceleration", train.deceleration}};
  }

  std::ofstream file(p / "trains.json");
  file << j << std::endl;
}

cda_rail::TrainList::TrainList(const std::filesystem::path& p) {
  /**
   * Construct object and read trains from file
   */

  if (!std::filesystem::is_directory(p)) {
    throw std::invalid_argument("Path is not a directory.");
  }
  if (!std::filesystem::exists(p)) {
    throw std::invalid_argument("Path does not exist.");
  }

  // Read the file
  std::ifstream f((p / "trains.json"));
  json          data = json::parse(f);

  // Add the trains
  for (const auto& [name, train] : data.items()) {
    this->add_train(name, train["length"], train["max_speed"],
                    train["acceleration"], train["deceleration"]);
  }
}
