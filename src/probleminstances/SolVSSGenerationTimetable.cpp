#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"

cda_rail::instances::SolVSSGenerationTimetable::SolVSSGenerationTimetable(
    cda_rail::instances::VSSGenerationTimetable instance, int dt)
    : instance(std::move(instance)), dt(dt) {
  vss_pos = std::vector<std::vector<double>>(
      this->instance.const_n().number_of_edges());

  train_pos.reserve(this->instance.get_train_list().size());
  train_speed.reserve(this->instance.get_train_list().size());

  for (size_t tr = 0; tr < this->instance.get_train_list().size(); ++tr) {
    const auto tr_interval = this->instance.time_index_interval(tr, dt, true);
    const auto tr_interval_size = tr_interval.second - tr_interval.first + 1;
    train_pos.emplace_back(tr_interval_size, -1);
    train_speed.emplace_back(tr_interval_size, -1);
  }
}

double
cda_rail::instances::SolVSSGenerationTimetable::get_train_pos(size_t train_id,
                                                              int time) const {
  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  if (instance.get_schedule(train_id).t_0 > time ||
      instance.get_schedule(train_id).t_n < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt == 0) {
    const auto t_index = static_cast<size_t>(time / dt);
    return train_pos.at(train_id).at(t_index);
  }

  const auto t_1 = static_cast<size_t>(std::floor(time / dt));
  const auto t_2 = t_1 + 1;

  const auto x_1 = train_pos.at(train_id).at(t_1);
  const auto v_1 = train_speed.at(train_id).at(t_1);
  const auto x_2 = train_pos.at(train_id).at(t_2);
  const auto v_2 = train_speed.at(train_id).at(t_2);

  if (approx_equal(x_2 - x_1, 0.5 * dt * (v_1 + v_2))) {
    const auto a   = (v_2 - v_1) / dt;
    const auto tau = time - static_cast<double>(t_1) * dt;
    return x_1 + v_1 * tau + 0.5 * a * tau * tau;
  }

  throw exceptions::ConsistencyException(
      "Train " + std::to_string(train_id) + " is not scheduled at time " +
      std::to_string(time) + " and cannot be inferred by linear interpolation");
}

double cda_rail::instances::SolVSSGenerationTimetable::get_train_speed(
    size_t train_id, int time) const {
  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  if (instance.get_schedule(train_id).t_0 > time ||
      instance.get_schedule(train_id).t_n < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt == 0) {
    const auto t_index = static_cast<size_t>(time / dt);
    return train_speed.at(train_id).at(t_index);
  }

  const auto t_1 = static_cast<size_t>(std::floor(time / dt));
  const auto t_2 = t_1 + 1;

  const auto x_1 = train_pos.at(train_id).at(t_1);
  const auto v_1 = train_speed.at(train_id).at(t_1);
  const auto x_2 = train_pos.at(train_id).at(t_2);
  const auto v_2 = train_speed.at(train_id).at(t_2);

  if (approx_equal(x_2 - x_1, 0.5 * dt * (v_1 + v_2))) {
    const auto a   = (v_2 - v_1) / dt;
    const auto tau = time - static_cast<double>(t_1) * dt;
    return v_1 + a * tau;
  }

  throw exceptions::ConsistencyException(
      "Train " + std::to_string(train_id) + " is not scheduled at time " +
      std::to_string(time) + " and cannot be inferred by linear interpolation");
}

void cda_rail::instances::SolVSSGenerationTimetable::add_vss_pos(
    size_t edge_id, double pos, bool reverse_edge) {
  if (!instance.const_n().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  const auto& edge = instance.const_n().get_edge(edge_id);

  if (pos < 0 || pos > edge.length) {
    throw exceptions::ConsistencyException(
        "VSS position " + std::to_string(pos) + " is not on edge " +
        std::to_string(edge_id));
  }

  vss_pos.at(edge_id).emplace_back(pos);
  std::sort(vss_pos.at(edge_id).begin(), vss_pos.at(edge_id).end());

  if (reverse_edge) {
    const auto reverse_edge_index =
        instance.const_n().get_reverse_edge_index(edge_id);
    if (reverse_edge_index.has_value()) {
      vss_pos.at(reverse_edge_index.value()).emplace_back(edge.length - pos);
      std::sort(vss_pos.at(reverse_edge_index.value()).begin(),
                vss_pos.at(reverse_edge_index.value()).end());
    }
  }
}

void cda_rail::instances::SolVSSGenerationTimetable::set_vss_pos(
    size_t edge_id, std::vector<double> pos) {
  if (!instance.const_n().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  const auto& edge = instance.const_n().get_edge(edge_id);

  for (const auto& p : pos) {
    if (p < 0 || p > edge.length) {
      throw exceptions::ConsistencyException(
          "VSS position " + std::to_string(p) + " is not on edge " +
          std::to_string(edge_id));
    }
  }

  vss_pos.at(edge_id) = std::move(pos);
}

void cda_rail::instances::SolVSSGenerationTimetable::reset_vss_pos(
    size_t edge_id) {
  if (!instance.const_n().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  vss_pos.at(edge_id).clear();
}

void cda_rail::instances::SolVSSGenerationTimetable::add_train_pos(
    size_t train_id, int time, double pos) {
  if (pos < 0) {
    throw exceptions::ConsistencyException(
        "Train position " + std::to_string(pos) + " is negative");
  }

  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  if (instance.time_interval(train_id).first > time ||
      instance.time_interval(train_id).second < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt != 0) {
    throw exceptions::ConsistencyException(
        "Time " + std::to_string(time) +
        " is not a multiple of dt = " + std::to_string(dt));
  }

  const auto tr_t0   = instance.time_index_interval(train_id, dt, true).first;
  const auto t_index = static_cast<size_t>(time / dt) - tr_t0;
  train_pos.at(train_id).at(t_index) = pos;
}

void cda_rail::instances::SolVSSGenerationTimetable::add_train_speed(
    size_t train_id, int time, double speed) {
  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  if (speed < 0) {
    throw exceptions::ConsistencyException(
        "Train speed " + std::to_string(speed) + " is negative");
  }
  if (speed > instance.get_train_list().get_train(train_id).max_speed) {
    throw exceptions::ConsistencyException(
        "Train speed " + std::to_string(speed) +
        " is greater than the maximum speed of train " +
        std::to_string(train_id) + " (" +
        std::to_string(
            instance.get_train_list().get_train(train_id).max_speed) +
        ")");
  }

  if (instance.time_interval(train_id).first > time ||
      instance.time_interval(train_id).second < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt != 0) {
    throw exceptions::ConsistencyException(
        "Time " + std::to_string(time) +
        " is not a multiple of dt = " + std::to_string(dt));
  }

  const auto tr_t0   = instance.time_index_interval(train_id, dt, true).first;
  const auto t_index = static_cast<size_t>(time / dt) - tr_t0;
  train_speed.at(train_id).at(t_index) = speed;
}

bool cda_rail::instances::SolVSSGenerationTimetable::check_consistency() const {
  if (status == SolutionStatus::Unknown) {
    return false;
  }
  if (obj < 0) {
    return false;
  }
  if (dt < 0) {
    return false;
  }
  if (!instance.check_consistency(true)) {
    return false;
  }
  for (const auto& train_pos_vec : train_pos) {
    for (const auto& pos : train_pos_vec) {
      if (pos < 0) {
        return false;
      }
    }
  }
  for (size_t tr_id = 0; tr_id < train_speed.size(); ++tr_id) {
    const auto& train = instance.get_train_list().get_train(tr_id);
    for (double t : train_speed.at(tr_id)) {
      if (t < 0 || t > train.max_speed) {
        return false;
      }
    }
  }
  for (size_t edge_id = 0; edge_id < vss_pos.size(); ++edge_id) {
    const auto& edge = instance.const_n().get_edge(edge_id);
    for (const auto& pos : vss_pos.at(edge_id)) {
      if (pos < 0 || pos > edge.length) {
        return false;
      }
    }
  }
  return true;
}
