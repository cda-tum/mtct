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
   * TrainList class
   */
private:
  std::vector<Train>                      trains;
  std::unordered_map<std::string, size_t> train_name_to_index;

public:
  // Constructors
  TrainList() = default;

  explicit TrainList(const std::filesystem::path& p);
  explicit TrainList(const std::string& path)
      : TrainList(std::filesystem::path(path)) {};
  explicit TrainList(const char* path)
      : TrainList(std::filesystem::path(path)) {};

  // Rule of 5
  TrainList(const TrainList& other)                = default;
  TrainList(TrainList&& other) noexcept            = default;
  TrainList& operator=(const TrainList& other)     = default;
  TrainList& operator=(TrainList&& other) noexcept = default;
  ~TrainList()                                     = default;

  // Iterators (for range-based for loops) that do not allow modification of the
  // underlying data
  [[nodiscard]] auto begin() const { return trains.begin(); };
  [[nodiscard]] auto end() const { return trains.end(); };
  [[nodiscard]] auto rbegin() const { return trains.rbegin(); };
  [[nodiscard]] auto rend() const { return trains.rend(); };

  size_t add_train(const std::string& name, int length, double max_speed,
                   double acceleration, double deceleration, bool tim = true);
  size_t add_train(const Train& train);
  [[nodiscard]] size_t size() const { return trains.size(); };

  [[nodiscard]] size_t       get_train_index(const std::string& name) const;
  [[nodiscard]] const Train& get_train(size_t index) const;
  [[nodiscard]] const Train& get_train(const std::string& name) const {
    return get_train(get_train_index(name));
  };

  [[nodiscard]] Train& editable_tr(size_t index);
  [[nodiscard]] Train& editable_tr(const std::string& name) {
    return editable_tr(get_train_index(name));
  };

  [[nodiscard]] bool has_train(const std::string& name) const {
    return train_name_to_index.find(name) != train_name_to_index.end();
  };
  [[nodiscard]] bool has_train(size_t index) const {
    return (index < trains.size());
  };

  void export_trains(const std::string& path) const {
    export_trains(std::filesystem::path(path));
  };
  void export_trains(const char* path) const {
    export_trains(std::filesystem::path(path));
  };
  void export_trains(const std::filesystem::path& p) const;
  [[nodiscard]] static TrainList import_trains(const std::string& path) {
    return TrainList(path);
  };
  [[nodiscard]] static TrainList import_trains(const char* path) {
    return TrainList(path);
  };
  [[nodiscard]] static TrainList import_trains(const std::filesystem::path& p) {
    return TrainList(p);
  };
};
} // namespace cda_rail
