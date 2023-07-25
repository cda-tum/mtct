#include "datastructure/Train.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <fstream>
#include <string>

using json = nlohmann::json;

size_t cda_rail::TrainList::add_train(const std::string& name, int length,
                                      double max_speed, double acceleration,
                                      double deceleration) {
  /**
   * Add a train to the list of trains.
   *
   * @param name The name of the train.
   * @param length The length of the train in m.
   * @param max_speed The maximum speed of the train in m/s.
   * @param acceleration The acceleration of the train in m/s^2.
   * @param deceleration The deceleration of the train in m/s^2.
   *
   * @return The index of the train in the list of trains.
   */
  if (has_train(name)) {
    throw exceptions::ConsistencyException("Train already exists.");
  }
  trains.emplace_back(name, length, max_speed, acceleration, deceleration);
  train_name_to_index[name] = trains.size() - 1;
  return train_name_to_index[name];
}

size_t cda_rail::TrainList::get_train_index(const std::string& name) const {
  /**
   * Returns the index of the train with the given name.
   *
   * @param name The name of the train.
   *
   * @return The index of the train with the given name.
   */
  if (!has_train(name)) {
    throw exceptions::TrainNotExistentException(name);
  }
  return train_name_to_index.at(name);
}

const cda_rail::Train& cda_rail::TrainList::get_train(size_t index) const {
  /**
   * Returns the train with the given index.
   *
   * @param index The index of the train.
   *
   * @return The train with the given index.
   */
  if (!has_train(index)) {
    throw exceptions::TrainNotExistentException(index);
  }
  return trains.at(index);
}

void cda_rail::TrainList::export_trains(const std::filesystem::path& p) const {
  /**
   * This method exports all trains to a directory in trains.json.
   *
   * @param p The path to the directory to export to.
   */
  if (!is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
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
    throw exceptions::ImportException("Path is not a directory.");
  }
  if (!std::filesystem::exists(p)) {
    throw exceptions::ImportException("Path does not exist.");
  }

  std::ifstream f((p / "trains.json"));
  json          data = json::parse(f);

  for (const auto& [name, train] : data.items()) {
    this->add_train(name, train["length"], train["max_speed"],
                    train["acceleration"], train["deceleration"]);
  }
}
