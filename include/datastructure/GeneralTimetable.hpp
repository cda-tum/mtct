#pragma once

#include <string>
#include <utility>

namespace cda_rail {
struct GeneralScheduledStop {
  /**
   * A (general) scheduled stop.
   */
  std::pair<int, int> begin;
  std::pair<int, int> end;
  int                 min_stopping_time;
  std::string         station;

  bool operator<(const GeneralScheduledStop& other) const {
    return (end.second < other.begin.first);
  }
  bool operator>(const GeneralScheduledStop& other) const {
    return (begin.first > other.end.second);
  }
  bool operator==(const GeneralScheduledStop& other) const {
    return (begin == other.begin && end == other.end);
  }
  bool operator<=(const GeneralScheduledStop& other) const {
    return *this < other || *this == other;
  }
  bool operator>=(const GeneralScheduledStop& other) const {
    return *this > other || *this == other;
  }
  bool operator!=(const GeneralScheduledStop& other) const {
    return !(*this == other);
  }

  // Constructor
  GeneralScheduledStop(std::pair<int, int> begin, std::pair<int, int> end,
                       int min_stopping_time, std::string station)
      : begin(std::move(begin)), end(std::move(end)),
        min_stopping_time(min_stopping_time), station(std::move(station)) {}
};

} // namespace cda_rail
