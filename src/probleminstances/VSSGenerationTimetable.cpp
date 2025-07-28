#include "probleminstances/VSSGenerationTimetable.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "datastructure/RailwayNetwork.hpp"

#include <cstddef>
#include <numeric>

using std::size_t;

void cda_rail::instances::VSSGenerationTimetable::discretize(
    const vss::SeparationFunction& sep_func) {
  /**
   * This method discretizes the network. It updates the timetable and the
   * routes accordingly.
   *
   * @param separation_type the type of separation to be used
   */

  const auto new_edges = this->n().discretize(sep_func);
  this->editable_timetable().update_after_discretization(new_edges);
  this->editable_routes().update_after_discretization(new_edges);
}

cda_rail::index_vector
cda_rail::instances::VSSGenerationTimetable::trains_at_t(int t) const {
  /**
   * Returns a list of all trains present at time t.
   *
   * @param t the time
   * @return a list of all trains present at time t.
   */
  const auto             tr_number = get_train_list().size();
  cda_rail::index_vector trains_to_consider(tr_number);
  std::iota(trains_to_consider.begin(), trains_to_consider.end(), 0);
  return trains_at_t(t, trains_to_consider);
}

cda_rail::index_vector cda_rail::instances::VSSGenerationTimetable::trains_at_t(
    int t, const cda_rail::index_vector& trains_to_consider) const {
  /**
   * Returns a list of all trains present at time t. But only considers the
   * trains in trains_to_consider.
   *
   * @param t the time
   * @param trains_to_consider the trains to consider
   * @return a list of all trains present at time t. But only considers the
   * trains in trains_to_consider.
   */

  if (t < 0) {
    throw exceptions::InvalidInputException("t must be non-negative.");
  }
  for (auto tr : trains_to_consider) {
    if (!get_train_list().has_train(tr)) {
      throw exceptions::TrainNotExistentException();
    }
  }

  cda_rail::index_vector trains;
  for (const auto tr : trains_to_consider) {
    const auto& interval = this->get_timetable().time_interval(tr);
    if (interval.first <= t && t < interval.second) {
      trains.push_back(tr);
    }
  }

  return trains;
}
