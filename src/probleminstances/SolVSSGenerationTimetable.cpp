#include "Definitions.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"

cda_rail::instances::SolVSSGenerationTimetable::SolVSSGenerationTimetable(
    cda_rail::instances::VSSGenerationTimetable instance, int dt)
    : instance(std::move(instance)), dt(dt) {
  vss_pos = std::vector<std::vector<double>>(
      this->instance.const_n().breakable_edges().size());
  train_pos =
      std::vector<std::vector<double>>(this->instance.get_train_list().size());
  train_speed =
      std::vector<std::vector<double>>(this->instance.get_train_list().size());
}
