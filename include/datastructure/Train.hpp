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
   * Train struct with relevant properties
   * @param name Name of the train
   * @param length Length of the train (in m)
   * @param max_speed Maximum speed of the train (in m/s)
   * @param acceleration Acceleration of the train (in m/s^2)
   * @param deceleration Deceleration of the train (in m/s^2)
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
  // Constructor
  Train(std::string name, double length, double max_speed, double acceleration,
        double deceleration, bool tim = true);
  // Rule of 0 suffices

  /*
   * GETTER
   */
  [[nodiscard]] std::string get_name() const { return m_name; };
  [[nodiscard]] double      get_length() const { return m_length; };
  [[nodiscard]] double      get_max_speed() const { return m_max_speed; };
  [[nodiscard]] double      get_acceleration() const { return m_acceleration; };
  [[nodiscard]] double      get_deceleration() const { return m_deceleration; };
  [[nodiscard]] bool        has_tim() const { return m_tim; };

  /*
   * SETTER
   */
  void set_name(std::string name) { m_name = std::move(name); };
  void set_length(double const length) {
    cda_rail::exceptions::throw_if_negative(length, "Train length");
    m_length = length;
  };
  void set_max_speed(double const max_speed) {
    cda_rail::exceptions::throw_if_less_than(max_speed, MIN_NON_ZERO,
                                             "Train max speed");
    m_max_speed = max_speed;
  };
  void set_acceleration(double const acceleration) {
    cda_rail::exceptions::throw_if_less_than(acceleration, MIN_NON_ZERO,
                                             "Train acceleration");
    m_acceleration = acceleration;
  }
  void set_deceleration(double const deceleration) {
    cda_rail::exceptions::throw_if_less_than(deceleration, MIN_NON_ZERO,
                                             "Train deceleration");
    m_deceleration = deceleration;
  };
  void set_tim_value(bool const tim) { m_tim = tim; };
  void set_tim() { set_tim_value(true); };
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
