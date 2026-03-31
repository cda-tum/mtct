#pragma once
#include "CustomExceptions.hpp"
#include "Definitions.hpp"

#include <cstddef>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cda_rail {
class Train {
  /**
   * @brief Properties and characteristics of a train.
   *
   * Stores the name, length, maximum speed, and acceleration/deceleration
   * capabilities, as well as train integrity monitoring status.
   *
   * @invariant m_length >= 0 (non-negative).
   * @invariant m_max_speed >= MIN_NON_ZERO (strictly positive).
   * @invariant m_acceleration >= MIN_NON_ZERO (strictly positive).
   * @invariant m_deceleration >= MIN_NON_ZERO (strictly positive).
   */
private:
  std::string m_name;         // name of train
  double      m_length;       // length of train in m: >= 0
  double      m_max_speed;    // maximal speed of train in m/s: >= MIN_NON_ZERO
  double      m_acceleration; // maximal acceleration in m/s^2: >= MIN_NON_ZERO
  double      m_deceleration; // maximal deceleration in m/s^2: >= MIN_NON_ZERO
  bool m_tim{true}; // indicates if train has a train integrity monitoring:
                    // default true

public:
  /**
   * @brief Constructs a train with specified dynamic properties.
   *
   * @pre length >= 0.
   * @pre max_speed >= MIN_NON_ZERO.
   * @pre acceleration >= MIN_NON_ZERO.
   * @pre deceleration >= MIN_NON_ZERO.
   * @param name Name identifier for the train.
   * @param length Length of the train in meters.
   * @param max_speed Maximum speed of the train in m/s.
   * @param acceleration Maximum acceleration capability in m/s².
   * @param deceleration Maximum deceleration capability in m/s².
   * @param tim Train integrity monitoring flag (default: true).
   * @throws cda_rail::exceptions::InvalidInputException If any numeric
   * precondition is violated.
   */
  Train(std::string name, double length, double max_speed, double acceleration,
        double deceleration, bool tim = true);
  // Rule of 0 suffices

  /*
   * GETTER
   */
  /**
   * @brief Returns the name of the train.
   *
   * @return Train name.
   */
  [[nodiscard]] std::string get_name() const { return m_name; };

  /**
   * @brief Returns the length of the train.
   *
   * @return Train length in meters (guaranteed >= 0).
   */
  [[nodiscard]] double get_length() const { return m_length; };

  /**
   * @brief Returns the maximum speed of the train.
   *
   * @return Maximum speed in m/s (guaranteed >= MIN_NON_ZERO).
   */
  [[nodiscard]] double get_max_speed() const { return m_max_speed; };

  /**
   * @brief Returns the maximum acceleration of the train.
   *
   * @return Maximum acceleration in m/s² (guaranteed >= MIN_NON_ZERO).
   */
  [[nodiscard]] double get_acceleration() const { return m_acceleration; };

  /**
   * @brief Returns the maximum deceleration of the train.
   *
   * @return Maximum deceleration in m/s² (guaranteed >= MIN_NON_ZERO).
   */
  [[nodiscard]] double get_deceleration() const { return m_deceleration; };

  /**
   * @brief Returns whether the train has train integrity monitoring.
   *
   * @return `true` if train has TIM enabled, `false` otherwise.
   */
  [[nodiscard]] bool has_tim() const { return m_tim; };

  /*
   * SETTER
   */
  /**
   * @brief Sets the name of the train.
   *
   * @param name New name for the train.
   */
  void set_name(std::string name) { m_name = std::move(name); };

  /**
   * @brief Sets the length of the train.
   *
   * @pre length >= 0.
   * @param length New length in meters.
   * @throws cda_rail::exceptions::InvalidInputException If `length` is
   * negative.
   */
  void set_length(double const length) {
    cda_rail::exceptions::throw_if_negative(length, "Train length");
    m_length = length;
  };

  /**
   * @brief Sets the maximum speed of the train.
   *
   * @pre max_speed >= MIN_NON_ZERO.
   * @param max_speed New maximum speed in m/s.
   * @throws cda_rail::exceptions::InvalidInputException If `max_speed` is less
   * than MIN_NON_ZERO.
   */
  void set_max_speed(double const max_speed) {
    cda_rail::exceptions::throw_if_less_than(max_speed, MIN_NON_ZERO,
                                             "Train max speed");
    m_max_speed = max_speed;
  };

  /**
   * @brief Sets the maximum acceleration of the train.
   *
   * @pre acceleration >= MIN_NON_ZERO.
   * @param acceleration New maximum acceleration in m/s².
   * @throws cda_rail::exceptions::InvalidInputException If `acceleration` is
   * less than MIN_NON_ZERO.
   */
  void set_acceleration(double const acceleration) {
    cda_rail::exceptions::throw_if_less_than(acceleration, MIN_NON_ZERO,
                                             "Train acceleration");
    m_acceleration = acceleration;
  }

  /**
   * @brief Sets the maximum deceleration of the train.
   *
   * @pre deceleration >= MIN_NON_ZERO.
   * @param deceleration New maximum deceleration in m/s².
   * @throws cda_rail::exceptions::InvalidInputException If `deceleration` is
   * less than MIN_NON_ZERO.
   */
  void set_deceleration(double const deceleration) {
    cda_rail::exceptions::throw_if_less_than(deceleration, MIN_NON_ZERO,
                                             "Train deceleration");
    m_deceleration = deceleration;
  };

  /**
   * @brief Sets the train integrity monitoring flag to a specific value.
   *
   * @param tim `true` to enable TIM, `false` to disable.
   */
  void set_tim_value(bool const tim) { m_tim = tim; };

  /**
   * @brief Enables train integrity monitoring.
   */
  void set_tim() { set_tim_value(true); };

  /**
   * @brief Disables train integrity monitoring.
   */
  void set_no_tim() { set_tim_value(false); };
};

class TrainList {
  /**
   * @brief Container for trains with name-based indexing.
   *
   * Maintains a sequence of trains and a lookup map from train name to index.
   *
   * @invariant `train_name_to_index` contains exactly one entry per train in
   * `trains`.
   * @invariant For every train name `n`, `train_name_to_index.at(n)` is the
   * index of that train in `trains`.
   * @invariant All train names in `trains` are unique.
   */
private:
  std::vector<Train> trains{}; // list of trains
  std::unordered_map<std::string, size_t>
      train_name_to_index{}; // helper struct: for any train name store its
                             // index in the vector above

public:
  /*
   * CONSTRUCTORS
   */

  /**
   * @brief Constructs an empty train list.
   */
  TrainList() = default;

  /**
   * @brief Constructs a train list by importing `trains.json` from a directory.
   *
   * @param p Path to the directory containing `trains.json`.
   * @throws cda_rail::exceptions::ImportException If `p` is not an existing
   * directory.
   * @throws cda_rail::exceptions::ConsistencyException If imported train names
   * are duplicated.
   * @throws cda_rail::exceptions::InvalidInputException If imported train
   * attributes violate train constraints.
   */
  explicit TrainList(std::filesystem::path const& p);

  /**
   * @brief Constructs a train list by importing from a path string.
   *
   * @param path Path to the directory containing `trains.json`.
   */
  explicit TrainList(std::string const& path)
      : TrainList(std::filesystem::path(path)) {};

  /**
   * @brief Constructs a train list by importing from a C-string path.
   *
   * @param path Path to the directory containing `trains.json`.
   */
  explicit TrainList(char const* const path)
      : TrainList(std::filesystem::path(path)) {};

  // Rule of 0 suffices

  /*
   * ITERATORS (for range-based for loops) that do not allow modification of the
   * underlying data
   */

  /**
   * @brief Returns a const iterator to the first train.
   *
   * @return Const iterator to the first element.
   */
  [[nodiscard]] constexpr auto cbegin() const { return trains.cbegin(); };

  /**
   * @brief Returns a const iterator to one-past-the-last train.
   *
   * @return Const iterator to the end sentinel.
   */
  [[nodiscard]] constexpr auto cend() const { return trains.cend(); };

  /**
   * @brief Returns a const reverse iterator to the last train.
   *
   * @return Const reverse iterator to the first reverse element.
   */
  [[nodiscard]] constexpr auto crbegin() const { return trains.crbegin(); };

  /**
   * @brief Returns a const reverse iterator to one-before-the-first train.
   *
   * @return Const reverse iterator to the reverse end sentinel.
   */
  [[nodiscard]] constexpr auto crend() const { return trains.crend(); };

  /**
   * @brief Returns the number of trains.
   *
   * @return Number of trains in the list.
   */
  [[nodiscard]] size_t size() const { return get_number_of_trains(); };

  /*
   * GETTER
   */

  /**
   * @brief Returns the index of a train by name.
   *
   * @param name Train name.
   * @return Index of the train in the internal vector.
   * @throws cda_rail::exceptions::TrainNotExistentException If no train with
   * `name` exists.
   */
  [[nodiscard]] size_t get_train_index(const std::string& name) const;

  /**
   * @brief Returns a train by index.
   *
   * @param index Index of the train.
   * @return Constant reference to the requested train.
   * @throws cda_rail::exceptions::TrainNotExistentException If `index` is out
   * of range.
   */
  [[nodiscard]] Train const& get_train(size_t index) const;

  /**
   * @brief Returns a train by name.
   *
   * @param name Train name.
   * @return Constant reference to the requested train.
   * @throws cda_rail::exceptions::TrainNotExistentException If no train with
   * `name` exists.
   */
  [[nodiscard]] Train const& get_train(std::string const& name) const {
    return get_train(get_train_index(name));
  };

  /**
   * @brief Returns a train by index. The train object is returned by reference
   * and can be edited.
   *
   * @param index Index of the train.
   * @return Reference to the requested train.
   * @throws cda_rail::exceptions::TrainNotExistentException If `index` is out
   * of range.
   */
  [[nodiscard]] Train& editable_train(size_t index) {
    if (!has_train(index)) {
      throw exceptions::TrainNotExistentException(index);
    }
    return trains.at(index);
  };

  /**
   * @brief Returns a train by name. The train object is returned by reference
   * and can be edited.
   *
   * @param name Train name.
   * @return Reference to the requested train.
   * @throws cda_rail::exceptions::TrainNotExistentException If `index` is out
   * of range.
   */
  [[nodiscard]] Train& editable_train(std::string const& name) {
    return editable_train(get_train_index(name));
  }

  /**
   * @brief Returns the number of trains.
   *
   * @return Number of trains in the list.
   */
  [[nodiscard]] size_t get_number_of_trains() const { return trains.size(); };

  /**
   * @brief Checks whether a train with the given name exists.
   *
   * @param name Train name.
   * @return `true` if a train with `name` exists, otherwise `false`.
   */
  [[nodiscard]] bool has_train(const std::string& name) const {
    return train_name_to_index.contains(name);
  };

  /**
   * @brief Checks whether a train index is valid.
   *
   * @param index Train index.
   * @return `true` if `index` refers to an existing train, otherwise `false`.
   */
  [[nodiscard]] bool has_train(size_t const index) const {
    return (index < get_number_of_trains());
  };

  /*
   * TRAIN SETTERS
   */

  /**
   * @brief Creates and adds a train from its attributes.
   *
   * @param name Train name.
   * @param length Train length in meters.
   * @param max_speed Maximum speed in m/s.
   * @param acceleration Maximum acceleration in m/s^2.
   * @param deceleration Maximum deceleration in m/s^2.
   * @param tim Train integrity monitoring flag.
   * @return Index of the added train.
   * @throws cda_rail::exceptions::ConsistencyException If a train with `name`
   * already exists.
   * @throws cda_rail::exceptions::InvalidInputException If train attributes are
   * invalid.
   */
  size_t add_train(std::string const& name, double const length,
                   double const max_speed, double const acceleration,
                   double const deceleration, bool const tim = true) {
    return add_train(
        {name, length, max_speed, acceleration, deceleration, tim});
  };

  /**
   * @brief Adds a train to the list.
   *
   * @param train Train to add.
   * @return Index of the added train.
   * @throws cda_rail::exceptions::ConsistencyException If a train with the
   * same name already exists.
   */
  size_t add_train(Train train);

  // No removal method for now (academic code): Is assumed that trains are only
  // added and no "mistakes" are made.

  /*
   * EXPORT/IMPORT
   */

  /**
   * @brief Exports all trains to `trains.json` in the target directory.
   *
   * @param path Path to the export directory.
   */
  void export_trains(std::string const& path) const {
    export_trains(std::filesystem::path(path));
  };

  /**
   * @brief Exports all trains to `trains.json` in the target directory.
   *
   * @param path Path to the export directory.
   */
  void export_trains(char const* const path) const {
    export_trains(std::filesystem::path(path));
  };

  /**
   * @brief Exports all trains to `trains.json` in the target directory.
   *
   * Creates the directory if needed and writes one JSON object keyed by train
   * name.
   *
   * @param p Path to the export directory.
   * @throws cda_rail::exceptions::ExportException If the directory cannot be
   * created.
   */
  void export_trains(std::filesystem::path const& p) const;

  /**
   * @brief Imports a train list from a path string.
   *
   * @param path Path to the directory containing `trains.json`.
   * @return Imported train list.
   */
  [[nodiscard]] static TrainList import_trains(std::string const& path) {
    return TrainList(path);
  };

  /**
   * @brief Imports a train list from a C-string path.
   *
   * @param path Path to the directory containing `trains.json`.
   * @return Imported train list.
   */
  [[nodiscard]] static TrainList import_trains(char const* const path) {
    return TrainList(path);
  };

  /**
   * @brief Imports a train list from a filesystem path.
   *
   * @param p Path to the directory containing `trains.json`.
   * @return Imported train list.
   */
  [[nodiscard]] static TrainList import_trains(std::filesystem::path const& p) {
    return TrainList(p);
  };
};
} // namespace cda_rail
