#pragma once
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace cda_rail {
struct Train {
  /**
   * Train struct with relevant properties
   * @param name Name of the train
   * @param length Length of the train (in m)
   * @param max_speed Maximum speed of the train (in m/s)
   * @param acceleration Acceleration of the train (in m/s^2)
   * @param deceleration Deceleration of the train (in m/s^2)
   */

  std::string name;
  int         length;
  double      max_speed;
  double      acceleration;
  double      deceleration;

  // Constructors
  Train() = default;
  Train(const std::string& name, int length, double max_speed,
        double acceleration, double deceleration)
      : name(name), length(length), max_speed(max_speed),
        acceleration(acceleration), deceleration(deceleration){};
};

class TrainList {
  /**
   * TrainList class
   */
private:
  std::vector<Train>                   trains;
  std::unordered_map<std::string, int> train_name_to_index;

public:
  // Constructors
  TrainList() = default;
  TrainList(const std::filesystem::path& p);
  TrainList(const std::string& path) : TrainList(std::filesystem::path(path)){};
  TrainList(const char* path) : TrainList(std::filesystem::path(path)){};

  // Rule of 5
  TrainList(const TrainList& other)                = default;
  TrainList(TrainList&& other) noexcept            = default;
  TrainList& operator=(const TrainList& other)     = default;
  TrainList& operator=(TrainList&& other) noexcept = default;
  ~TrainList()                                     = default;

  // Iterators (for range-based for loops) that do not allow modification of the
  // underlying data
  auto begin() const { return trains.begin(); };
  auto end() const { return trains.end(); };
  auto rbegin() const { return trains.rbegin(); };
  auto rend() const { return trains.rend(); };

  int add_train(const std::string& name, int length, double max_speed,
                double acceleration, double deceleration);
  [[nodiscard]] int size() const { return trains.size(); };

  [[nodiscard]] int          get_train_index(const std::string& name) const;
  [[nodiscard]] const Train& get_train(int index) const;
  [[nodiscard]] const Train& get_train(const std::string& name) const {
    return get_train(get_train_index(name));
  };

  [[nodiscard]] bool has_train(const std::string& name) const {
    return train_name_to_index.find(name) != train_name_to_index.end();
  };
  [[nodiscard]] bool has_train(int index) const {
    return (index >= 0 && index < trains.size());
  };

  void export_trains(const std::string& path) const {
    export_trains(std::filesystem::path(path));
  };
  void export_trains(const char* path) const {
    export_trains(std::filesystem::path(path));
  };
  void export_trains(const std::filesystem::path& p) const;
  [[nodiscard]] static cda_rail::TrainList
  import_trains(const std::string& path) {
    return TrainList(path);
  };
  [[nodiscard]] static cda_rail::TrainList import_trains(const char* path) {
    return TrainList(path);
  };
  [[nodiscard]] static cda_rail::TrainList
  import_trains(const std::filesystem::path& p) {
    return TrainList(p);
  };
};
} // namespace cda_rail
