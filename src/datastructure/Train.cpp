#include "datastructure/Train.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>

using json = nlohmann::json;

/*
 * TRAIN
 */
cda_rail::Train::Train(std::string name, double const length,
                       double const max_speed, double const acceleration,
                       double const deceleration, bool const tim)
    : m_name(std::move(name)), m_length(length), m_max_speed(max_speed),
      m_acceleration(acceleration), m_deceleration(deceleration), m_tim(tim) {
  cda_rail::exceptions::throw_if_negative(m_length, "Train length");
  cda_rail::exceptions::throw_if_less_than(m_max_speed, MIN_NON_ZERO,
                                           "Train max speed");
  cda_rail::exceptions::throw_if_less_than(m_acceleration, MIN_NON_ZERO,
                                           "Train acceleration");
  cda_rail::exceptions::throw_if_less_than(m_deceleration, MIN_NON_ZERO,
                                           "Train deceleration");
}

/*
 * TRAIN LIST
 */

// CONSTRUCTOR

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
    const bool tim =
        train.contains("tim") ? static_cast<bool>(train["tim"]) : true;
    this->add_train(name, train["length"], train["max_speed"],
                    train["acceleration"], train["deceleration"], tim);
  }
}

// GETTER

size_t cda_rail::TrainList::get_train_index(std::string const& name) const {
  if (!has_train(name)) {
    throw exceptions::TrainNotExistentException(name);
  }
  return train_name_to_index.at(name);
}

const cda_rail::Train&
cda_rail::TrainList::get_train(size_t const index) const {
  if (!has_train(index)) {
    throw exceptions::TrainNotExistentException(index);
  }
  return trains.at(index);
}

// SETTER

size_t cda_rail::TrainList::add_train(Train train) {
  if (has_train(train.get_name())) {
    throw exceptions::ConsistencyException("Train " + train.get_name() +
                                           " already exists.");
  }
  trains.push_back(std::move(train));
  size_t const idx                              = trains.size() - 1;
  train_name_to_index[trains.back().get_name()] = idx;
  return idx;
}

// EXPORT

void cda_rail::TrainList::export_trains(std::filesystem::path const& p) const {
  if (!is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  json j;
  for (const auto& train : trains) {
    j[train.get_name()] = {{"length", train.get_length()},
                           {"max_speed", train.get_max_speed()},
                           {"acceleration", train.get_acceleration()},
                           {"deceleration", train.get_acceleration()},
                           {"tim", train.has_tim()}};
  }

  std::ofstream file(p / "trains.json");
  file << j << '\n';
}
