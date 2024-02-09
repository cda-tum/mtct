#include "datastructure/Timetable.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/Station.hpp"
#include "nlohmann/json.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

using json = nlohmann::json;

std::pair<size_t, size_t>
cda_rail::Timetable::time_index_interval(size_t train_index, int dt,
                                         bool tn_inclusive) const {
  /**
   * This method returns the time interval of a train schedule as indices given
   * a time step length dt.
   *
   * @param train_index The index of the train in the train list.
   * @param dt The time step length.
   * @return A pair of integers (t_0, t_n) where t_0 is the time index at which
   * the train enters the network and t_n is the time index at which the train
   * leaves the network.
   */

  if (!train_list.has_train(train_index)) {
    throw exceptions::TrainNotExistentException(train_index);
  }

  const auto& schedule = schedules.at(train_index);
  const auto& t_0      = schedule.get_t_0();
  const auto& t_n      = schedule.get_t_n();

  if (t_0 < 0 || t_n < 0) {
    throw exceptions::ConsistencyException("Time cannot be negative.");
  }

  const auto t_0_index = t_0 / dt;
  const auto t_n_index =
      (t_n % dt == 0 ? t_n / dt - 1 : t_n / dt) + (tn_inclusive ? 1 : 0);

  return {static_cast<size_t>(t_0_index), static_cast<size_t>(t_n_index)};
}
