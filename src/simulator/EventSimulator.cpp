// The event-based simulator is based on the work of Sebastian Ludwig in his
// bachelor's thesis "Event-Based Simulation of Modern Railway Networks",
// Technical University of Munich, 2026. From there stem the ideas of planning
// each train's fastest movement up to the end of its movement authority, of
// processing the trains in a priority queue of events, and of letting a train
// that waits for another one be woken up by it.

#include "simulator/EventSimulator.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"
#include "plog/Log.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GeneralSimulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// The reinterpret_cast warnings are false positives stemming from the plog
// macros.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

namespace {
// How often a simulation may handle events at the same time per train before
// it is considered stuck
constexpr size_t SAME_TIME_EVENT_LIMIT = 1000;

// The time needed to cover a distance starting with speed v_0 and accelerating
// constantly, in a form that is numerically stable also for small accelerations
double time_to_cover(double v_0, double acceleration, double distance) {
  const auto denominator =
      v_0 +
      std::sqrt(std::max(0.0, (v_0 * v_0) + (2 * acceleration * distance)));
  return denominator > 0 ? 2 * distance / denominator : 0.0;
}

// The boundary of a predicate that holds at lo, possibly up to some point, and
// not beyond, as the largest point found to hold and the smallest found not to
template <typename Predicate>
std::pair<double, double> bisect(double lo, double hi, Predicate const& holds) {
  for (int i = 0; i < 100; ++i) {
    const auto mid = (lo + hi) / 2;
    if (holds(mid)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return {lo, hi};
}
} // namespace

// ----------------
// CONSTRUCTOR

cda_rail::simulator::EventSimulator::EventSimulator(
    cda_rail::instances::GeneralPerformanceOptimizationInstance const& instance,
    std::vector<cda_rail::index_set> ttd_sections)
    : GeneralSimulator(
          std::make_shared<
              const instances::GeneralPerformanceOptimizationInstance>(
              instance),
          std::move(ttd_sections)) {}

cda_rail::simulator::EventSimulator::EventSimulator(
    cda_rail::instances::GeneralPerformanceOptimizationInstance const& instance,
    std::vector<cda_rail::index_set>    ttd_sections,
    std::vector<cda_rail::index_vector> train_edges,
    std::vector<cda_rail::index_vector> ttd_orders,
    std::vector<cda_rail::index_vector> vertex_orders,
    std::vector<std::vector<double>>    stop_positions)
    : GeneralSimulator(
          std::make_shared<
              const instances::GeneralPerformanceOptimizationInstance>(
              instance),
          std::move(ttd_sections), std::move(train_edges),
          std::move(ttd_orders), std::move(vertex_orders),
          std::move(stop_positions)) {}

// ---------------
// MOVEMENT

double cda_rail::simulator::EventSimulator::Movement::end_time() const {
  double time = t;
  for (const auto& phase : phases) {
    time += phase.duration;
  }
  return time;
}

double cda_rail::simulator::EventSimulator::Movement::end_pos() const {
  double position = pos;
  for (const auto& phase : phases) {
    position += phase.distance();
  }
  return position;
}

cda_rail::simulator::PosVel
cda_rail::simulator::EventSimulator::Movement::at(double time) const {
  double start    = t;
  double position = pos;
  for (const auto& phase : phases) {
    if (time < start + phase.duration) {
      const auto tau = std::max(0.0, time - start);
      const auto v   = phase.v_start + (phase.acceleration() * tau);
      return {.pos = position + ((phase.v_start + v) * tau / 2), .vel = v};
    }
    start += phase.duration;
    position += phase.distance();
  }
  return {.pos = position, .vel = end_speed()};
}

double
cda_rail::simulator::EventSimulator::Movement::time_at(double position) const {
  if (position <= pos) {
    return t;
  }
  double start = t;
  double x     = pos;
  for (const auto& phase : phases) {
    if (position <= x + phase.distance()) {
      return start +
             time_to_cover(phase.v_start, phase.acceleration(), position - x);
    }
    start += phase.duration;
    x += phase.distance();
  }
  return INF;
}

double cda_rail::simulator::EventSimulator::Movement::braking_point_time(
    double position, double d) const {
  const auto braking_point = [d](double x, double v) {
    return x + (v * v / (2 * d));
  };
  double start = t;
  double x     = pos;
  for (const auto& phase : phases) {
    const auto braking_point_start = braking_point(x, phase.v_start);
    if (braking_point_start >= position) {
      return start;
    }
    // Within a phase, the braking point moves (1 + acceleration / d) times as
    // far as the front
    const auto factor = 1 + (phase.acceleration() / d);
    if (factor > 0 &&
        braking_point_start + (factor * phase.distance()) >= position) {
      return start + time_to_cover(phase.v_start, phase.acceleration(),
                                   (position - braking_point_start) / factor);
    }
    start += phase.duration;
    x += phase.distance();
  }
  return braking_point(x, end_speed()) >= position ? start : INF;
}

double
cda_rail::simulator::EventSimulator::Movement::final_braking_start() const {
  if (phases.empty() || phases.back().v_end >= phases.back().v_start) {
    return end_time();
  }
  return end_time() - phases.back().duration;
}

cda_rail::simulator::EventSimulator::Movement
cda_rail::simulator::EventSimulator::Movement::until(double position) const {
  Movement result{.t = t, .pos = pos, .v = v, .phases = {}};
  double   x = pos;
  for (const auto& phase : phases) {
    if (position < x + phase.distance()) {
      if (position > x) {
        const auto tau =
            time_to_cover(phase.v_start, phase.acceleration(), position - x);
        result.phases.push_back(
            {.duration = tau,
             .v_start  = phase.v_start,
             .v_end    = phase.v_start + (phase.acceleration() * tau)});
      }
      return result;
    }
    result.phases.push_back(phase);
    x += phase.distance();
  }
  return result;
}

std::vector<cda_rail::simulator::EventSimulator::Phase>
cda_rail::simulator::EventSimulator::fastest_phases(
    double pos, double v_0, double target, double v_target,
    std::vector<SpeedLimit> const& limits, double a, double d) {
  struct Section {
    double start;
    double end;
    double v2_max;
  };
  std::vector<Section> sections;
  for (const auto& [end, v_max] : limits) {
    const auto start = sections.empty() ? pos : sections.back().end;
    if (start >= target) {
      break;
    }
    sections.push_back({.start  = start,
                        .end    = std::min(end, target),
                        .v2_max = v_max * v_max});
  }

  // The highest squared speed at the end of every section from which the
  // train can still obey all later speed limits and its target speed
  std::vector<double> v2_end(sections.size());
  double              v2_next = v_target < INF ? v_target * v_target : INF;
  for (size_t k = sections.size(); k-- > 0;) {
    v2_end.at(k) = std::min(sections.at(k).v2_max, v2_next);
    v2_next      = std::min(
        sections.at(k).v2_max,
        v2_end.at(k) + (2 * d * (sections.at(k).end - sections.at(k).start)));
  }

  // Consecutive pieces in which the train does the same form one phase, whose
  // duration follows from its length to avoid summing up rounding errors
  enum class Action : std::uint8_t { Accelerate, Cruise, Brake };
  std::vector<Phase>    phases;
  std::optional<Action> last_action;
  double                phase_start = pos;
  const auto add_piece = [&](double start, double end, double v_s, double v_e,
                             Action action) {
    if (last_action == action) {
      phases.back().v_end = v_e;
    } else {
      phases.push_back({.duration = 0, .v_start = v_s, .v_end = v_e});
      last_action = action;
      phase_start = start;
    }
    phases.back().duration =
        2 * (end - phase_start) / (phases.back().v_start + v_e);
  };
  double v2 = v_0 * v_0;
  for (size_t k = 0; k < sections.size(); ++k) {
    const auto length = sections.at(k).end - sections.at(k).start;
    const auto v2_max = sections.at(k).v2_max;
    // The squared speed when accelerating fully from the section's start and
    // when braking fully for what comes after it
    const auto accelerating = [&](double u) { return v2 + (2 * a * u); };
    const auto braking      = [&](double u) {
      return v2_end.at(k) + (2 * d * (length - u));
    };
    const auto v2_at = [&](double u) {
      return std::min({v2_max, accelerating(u), braking(u)});
    };
    const auto action_at = [&](double u) {
      if (accelerating(u) <= v2_at(u)) {
        return Action::Accelerate;
      }
      return v2_max <= braking(u) ? Action::Cruise : Action::Brake;
    };

    // The minimum of the three is linear in between the points at which two of
    // them are equal
    std::vector<double> kinks{0, length, (v2_max - v2) / (2 * a),
                              length - ((v2_max - v2_end.at(k)) / (2 * d)),
                              (braking(0) - v2) / (2 * (a + d))};
    for (auto& u : kinks) {
      u = std::clamp(u, 0.0, length);
    }
    std::ranges::sort(kinks);
    for (size_t j = 0; j + 1 < kinks.size(); ++j) {
      const auto u_0 = kinks.at(j);
      const auto u_1 = kinks.at(j + 1);
      const auto v_s = std::sqrt(v2_at(u_0));
      const auto v_e = std::sqrt(v2_at(u_1));
      if (u_1 - u_0 < EPS || v_s + v_e <= 0) {
        continue;
      }
      add_piece(sections.at(k).start + u_0, sections.at(k).start + u_1, v_s,
                v_e, action_at((u_0 + u_1) / 2));
    }
    v2 = v2_at(length);
  }
  return phases;
}

std::vector<cda_rail::simulator::EventSimulator::SpeedLimit>
cda_rail::simulator::EventSimulator::speed_limits(
    size_t tr, double from, double to,
    bool limit_speed_by_leaving_edges) const {
  const auto& train      = get_instance()->get_const_train_list().get_train(tr);
  const auto& network    = get_instance()->get_const_network();
  const auto& edges      = get_train_edges_of_tr(tr);
  const auto  milestones = edge_milestones(tr);
  // An edge limits the speed while the front is between the edge's start and
  // its end plus reach
  const auto reach = limit_speed_by_leaving_edges ? train.get_length() : 0.0;

  std::vector<double> ends{to};
  for (size_t i = 0; i < edges.size(); ++i) {
    for (const auto p : {milestones.at(i), milestones.at(i + 1) + reach}) {
      if (from < p && p < to) {
        ends.push_back(p);
      }
    }
  }
  std::ranges::sort(ends);

  std::vector<SpeedLimit> limits;
  double                  start = from;
  for (const auto end : ends) {
    if (end <= start) {
      continue;
    }
    const auto mid   = (start + end) / 2;
    double     v_max = train.get_max_speed();
    for (size_t i = 0; i < edges.size(); ++i) {
      if (milestones.at(i) < mid && mid < milestones.at(i + 1) + reach) {
        v_max = std::min(v_max, network.get_edge(edges.at(i)).max_speed);
      }
    }
    if (mid > milestones.back()) {
      v_max = std::min(v_max, network.get_edge(edges.back()).max_speed);
    }
    if (!limits.empty() && approx_equal(limits.back().v_max, v_max)) {
      limits.back().end = end;
    } else {
      limits.push_back({.end = end, .v_max = v_max});
    }
    start = end;
  }
  return limits;
}

cda_rail::simulator::EventSimulator::Movement
cda_rail::simulator::EventSimulator::exit_movement(
    size_t tr, double t, PosVel front,
    bool limit_speed_by_leaving_edges) const {
  const auto& train    = get_instance()->get_const_train_list().get_train(tr);
  const auto& schedule = get_instance()->get_const_schedule(tr);
  const auto  a        = train.get_acceleration();
  const auto  d        = train.get_deceleration();
  const auto  exit_vertex_pos = train_edge_length(tr);
  const auto  exit_pos        = exit_vertex_pos + train.get_length();
  const auto  exit_time       = schedule.get_exit_time();
  const auto  v_exit          = schedule.get_exit_velocity();
  const auto  v_min           = std::min(V_MIN, v_exit);

  // Continues a movement as fast as possible up to the target
  const auto continue_fastest = [&](Movement movement, double target,
                                    double v_target) {
    const auto pos    = movement.end_pos();
    const auto phases = fastest_phases(
        pos, movement.end_speed(), target, v_target,
        speed_limits(tr, pos, target, limit_speed_by_leaving_edges), a, d);
    movement.phases.insert(movement.phases.end(), phases.begin(), phases.end());
    return movement;
  };
  const Movement now{.t = t, .pos = front.pos, .v = front.vel, .phases = {}};
  auto           fastest = continue_fastest(now, exit_pos, v_exit);
  if (fastest.end_time() + GRB_EPS >= exit_time) {
    return fastest;
  }

  // Braking fully from position s of the fastest movement on, the train gets
  // to its exit position the later and the slower, the earlier it starts
  const auto arrival_speed2 = [&](double s) {
    const auto v = fastest.until(s).end_speed();
    return (v * v) - (2 * d * (exit_pos - s));
  };
  const auto late_braking = [&](double s) {
    auto       movement = fastest.until(s);
    const auto v_s      = movement.end_speed();
    const auto v_e      = std::sqrt(std::max(0.0, arrival_speed2(s)));
    if (v_s + v_e > 0) {
      movement.phases.push_back({.duration = 2 * (exit_pos - s) / (v_s + v_e),
                                 .v_start  = v_s,
                                 .v_end    = v_e});
    }
    return movement;
  };
  const auto too_slow = [&](double s) {
    return arrival_speed2(s) < v_min * v_min;
  };
  const auto can_brake = !too_slow(exit_pos);
  const auto earliest_braking =
      too_slow(front.pos) ? bisect(front.pos, exit_pos, too_slow).second
                          : front.pos;
  if (can_brake && late_braking(earliest_braking).end_time() >= exit_time) {
    return late_braking(bisect(earliest_braking, exit_pos, [&](double s) {
                          return late_braking(s).end_time() >= exit_time;
                        }).first);
  }

  // Otherwise, the train has to stop at its exit vertex, unless it is too
  // late for that
  if (front.pos + tr_braking_distance(tr, front.vel) >
      exit_vertex_pos + GRB_EPS) {
    return can_brake ? late_braking(earliest_braking) : fastest;
  }
  // It brakes towards its exit vertex until x and then leaves as fast as
  // possible at the lowest speed it may have outside the network
  const auto dip = [&](double x) {
    return continue_fastest(
        continue_fastest(now, x, std::sqrt(2 * d * (exit_vertex_pos - x))),
        exit_pos, v_min);
  };
  if (dip(exit_vertex_pos).end_time() >= exit_time) {
    return dip(bisect(front.pos, exit_vertex_pos, [&](double x) {
                 return dip(x).end_time() < exit_time;
               }).second);
  }
  // If even that is too early, it waits at its exit vertex until leaving as
  // fast as possible gets it out at its exit time
  auto       movement = continue_fastest(now, exit_vertex_pos, 0);
  const auto leaving  = continue_fastest(
      {.t = 0, .pos = exit_vertex_pos, .v = 0, .phases = {}}, exit_pos, v_exit);
  const auto wait = exit_time - leaving.end_time() - movement.end_time();
  if (wait > 0) {
    movement.phases.push_back({.duration = wait, .v_start = 0, .v_end = 0});
  }
  movement.phases.insert(movement.phases.end(), leaving.phases.begin(),
                         leaving.phases.end());
  return movement;
}

// ----------------
// SIMULATION RUN

// Everything a simulation keeps track of. Every train has a planned movement
// and an event at which it is simulated again. A train that waits for another
// one is woken up by that train whenever its plans change.
class cda_rail::simulator::EventSimulator::Run {
public:
  Run(EventSimulator const& simulator, double dt, bool late_entry_possible,
      bool limit_speed_by_leaving_edges, bool save_trajectories,
      bool block_vertices_after_disappearing);
  Run(Run const&)            = delete;
  Run(Run&&)                 = delete;
  Run& operator=(Run const&) = delete;
  Run& operator=(Run&&)      = delete;
  ~Run()                     = default;

  [[nodiscard]] SimulatorResults simulate();

private:
  enum class Status : std::uint8_t { Waiting, Driving, Left };
  enum class End : std::uint8_t { Obstacle, Stop, RouteEnd, Exit };

  // What keeps a train from moving on: another train, whose rear has to get to
  // rear_pos along that train's route or which has to leave the network, or a
  // point in time. The waiting train is not simulated again before not_before.
  struct Blocker {
    std::optional<size_t> train;
    double                rear_pos{-INF};
    double                not_before{-INF};
  };

  // Where a train's movement authority ends and why. Moreover, the train's
  // braking point must not pass checkpoint before it is checked again whether
  // the edge starting there is free.
  struct Target {
    double  pos;
    End     end;
    Blocker blocker{};
    bool    follows{false};
    double  checkpoint{INF};
  };

  // Whether a train occupies the reverse of an edge, and whether any other
  // train may still use it
  struct ReverseEdgeUse {
    std::optional<Blocker> blocker;
    bool                   used{false};
  };

  struct TrainState {
    Status                 status{Status::Waiting};
    Movement               movement{};
    std::optional<size_t>  next_stop;
    double                 stop_until{-INF};
    double                 blocked_pos{INF};
    std::optional<Blocker> blocker;
    double                 next_event{INF};
    size_t                 next_event_id{0};
  };

  // Events
  void process(size_t tr, double t);
  void try_to_enter(size_t tr, double t);
  void enter(size_t tr, double t);
  void serve_stop(size_t tr, size_t stop, double t);
  void leave(size_t tr, double t);
  void plan(size_t tr, double t);
  void block_last_vertex(size_t tr, double t);

  // Event queue
  void                 schedule(size_t tr, double t);
  void                 schedule_at_most(size_t tr, double t);
  void                 wake_trains_waiting_for(size_t tr, double t);
  [[nodiscard]] bool   is_released(Blocker const& blocker, double t) const;
  [[nodiscard]] double release_time(Blocker const& blocker, double t) const;

  // Movement authority
  [[nodiscard]] Target find_target(size_t tr, double t, PosVel now) const;
  [[nodiscard]] std::optional<Target>
  train_ahead(size_t tr, size_t i, double t, double pos, bool first_edge) const;
  [[nodiscard]] std::optional<Blocker> entry_blocker(size_t tr, double t) const;
  [[nodiscard]] std::optional<Blocker> ttd_blocker(size_t tr, size_t i,
                                                   double t) const;
  [[nodiscard]] ReverseEdgeUse         reverse_edge_use(size_t tr, size_t edge,
                                                        double t) const;
  [[nodiscard]] std::optional<Blocker>
  vertex_order_blocker(size_t tr, size_t vertex, double t) const;

  // Positions
  [[nodiscard]] std::optional<std::pair<double, double>>
                       position_on_edge(size_t tr, size_t edge, double t) const;
  [[nodiscard]] double ttd_end(size_t tr, size_t ttd) const;
  [[nodiscard]] std::optional<double> route_pos(size_t tr, size_t vertex) const;
  [[nodiscard]] bool                  leaves_network(size_t tr) const;
  [[nodiscard]] double                end_pos(size_t tr) const;
  [[nodiscard]] std::optional<size_t> stop_at_end(size_t tr) const;
  [[nodiscard]] double                length(size_t tr) const {
    return m_instance.get_const_train_list().get_train(tr).get_length();
  }
  [[nodiscard]] cda_rail::index_vector const& route(size_t tr) const {
    return m_simulator.get_train_edges_of_tr(tr);
  }
  [[nodiscard]] Network const& network() const {
    return m_instance.get_const_network();
  }
  [[nodiscard]] PosVel front(size_t tr, double t) const {
    return m_trains.at(tr).movement.at(t);
  }

  // Trajectories
  void record(size_t tr, double t);
  void remove_uninformative_samples(size_t tr);

  EventSimulator const&                                    m_simulator;
  instances::GeneralPerformanceOptimizationInstance const& m_instance;
  double                                                   m_dt;
  bool                             m_late_entry_possible;
  bool                             m_limit_speed_by_leaving_edges;
  bool                             m_save_trajectories;
  bool                             m_block_vertices;
  std::vector<std::vector<double>> m_milestones;
  std::vector<std::unordered_map<size_t, double>> m_edge_starts;
  std::vector<std::unordered_set<size_t>>         m_trains_on_edges;
  std::vector<TrainState>                         m_trains;
  std::vector<double>                             m_vertex_headways;
  std::vector<double>                             m_exit_times;
  std::vector<std::vector<double>>                m_stop_times;
  std::vector<std::map<double, PosVel>>           m_trajectories;
  // Every event is the time, the train, and the train's event id at the time
  // it was scheduled
  std::priority_queue<std::tuple<double, size_t, size_t>,
                      std::vector<std::tuple<double, size_t, size_t>>,
                      std::greater<>>
       m_events;
  bool m_failed{false};
};

cda_rail::simulator::EventSimulator::Run::Run(
    EventSimulator const& simulator, double dt, bool late_entry_possible,
    bool limit_speed_by_leaving_edges, bool save_trajectories,
    bool block_vertices_after_disappearing)
    : m_simulator(simulator), m_instance(*simulator.get_instance()), m_dt(dt),
      m_late_entry_possible(late_entry_possible),
      m_limit_speed_by_leaving_edges(limit_speed_by_leaving_edges),
      m_save_trajectories(save_trajectories),
      m_block_vertices(block_vertices_after_disappearing),
      m_trains_on_edges(simulator.tr_on_edges()),
      m_trains(m_instance.get_const_train_list().size()),
      m_vertex_headways(m_instance.get_const_network().number_of_vertices(),
                        0.0),
      m_exit_times(m_trains.size()), m_stop_times(m_trains.size()),
      m_trajectories(save_trajectories ? m_trains.size() : 0) {
  for (size_t tr = 0; tr < m_trains.size(); ++tr) {
    m_milestones.push_back(simulator.edge_milestones(tr));
    auto&       starts = m_edge_starts.emplace_back();
    const auto& edges  = route(tr);
    // An edge that a route contains twice starts where it is first reached
    for (size_t i = edges.size(); i-- > 0;) {
      starts[edges.at(i)] = m_milestones.back().at(i);
    }
    m_exit_times.at(tr) = m_instance.get_const_schedule(tr).get_entry_time();
  }
}

cda_rail::simulator::SimulatorResults
cda_rail::simulator::EventSimulator::Run::simulate() {
  for (size_t tr = 0; tr < m_trains.size(); ++tr) {
    if (!route(tr).empty()) {
      schedule(tr, m_instance.get_const_schedule(tr).get_entry_time());
    }
  }

  double last_t           = -INF;
  size_t same_time_events = 0;
  while (!m_failed && !m_events.empty()) {
    const auto [t, tr, id] = m_events.top();
    m_events.pop();
    if (id != m_trains.at(tr).next_event_id) {
      continue; // The train has been rescheduled since
    }
    // The events are handled in chronological order
    same_time_events = t <= last_t ? same_time_events + 1 : 0;
    last_t           = t;
    if (same_time_events > SAME_TIME_EVENT_LIMIT * m_trains.size()) {
      throw exceptions::ConsistencyException(
          "Event-based simulation does not advance in time.");
    }
    m_trains.at(tr).next_event = INF;
    process(tr, t);
  }

  // Otherwise, the remaining trains wait for each other
  const auto success =
      !m_failed &&
      std::ranges::all_of(
          std::views::iota(size_t{0}, m_trains.size()), [this](size_t tr) {
            return route(tr).empty() || m_trains.at(tr).status == Status::Left;
          });
  PLOGV << "Event-based simulation " << (success ? "succeeded" : "failed");
  for (size_t tr = 0; tr < m_trajectories.size(); ++tr) {
    remove_uninformative_samples(tr);
  }
  return {.success            = success,
          .exit_times         = std::move(m_exit_times),
          .stop_times         = std::move(m_stop_times),
          .vertex_headways    = std::move(m_vertex_headways),
          .train_trajectories = std::move(m_trajectories)};
}

// Events

void cda_rail::simulator::EventSimulator::Run::process(size_t tr, double t) {
  auto& state = m_trains.at(tr);
  if (state.status == Status::Waiting) {
    try_to_enter(tr, t);
    return;
  }
  if (front(tr, t).pos > state.blocked_pos + GRB_EPS) {
    PLOGV << "At time " << t << ", train " << tr
          << " passed a vertex blocked by a train that disappeared there.";
    m_failed = true;
    return;
  }
  if (t < state.stop_until) {
    schedule(tr, state.stop_until);
    return;
  }
  if (const auto stop = stop_at_end(tr);
      stop.has_value() && t >= state.movement.end_time()) {
    serve_stop(tr, *stop, t);
    return;
  }
  if (front(tr, t).pos > end_pos(tr) - GRB_EPS) {
    leave(tr, t);
    return;
  }
  plan(tr, t);
}

void cda_rail::simulator::EventSimulator::Run::try_to_enter(size_t tr,
                                                            double t) {
  const auto& schedule = m_instance.get_const_schedule(tr);
  if (!std::ranges::contains(
          m_simulator.get_vertex_orders_of_vertex(schedule.get_entry_vertex()),
          tr)) {
    return; // The train is not scheduled to enter at all
  }
  if (!m_late_entry_possible && t > schedule.get_entry_time()) {
    PLOGV << "Train " << tr << " cannot enter at its entry time.";
    m_failed = true;
    return;
  }
  const auto blocker = entry_blocker(tr, t);
  if (!blocker.has_value()) {
    enter(tr, t);
    return;
  }
  const auto release = release_time(*blocker, t);
  // Without late entries, a train can only wait for one whose plans are not
  // known yet, since it may enter at the same time
  if (!m_late_entry_possible && release > t && release < INF) {
    PLOGV << "Train " << tr << " cannot enter at its entry time.";
    m_failed = true;
    return;
  }
  m_trains.at(tr).blocker = blocker;
  this->schedule(tr, release);
}

void cda_rail::simulator::EventSimulator::Run::enter(size_t tr, double t) {
  auto&       state    = m_trains.at(tr);
  const auto& schedule = m_instance.get_const_schedule(tr);
  const auto  entry    = schedule.get_entry_vertex();
  PLOGV << "At time " << t << ", train " << tr << " enters the network.";
  state.status   = Status::Driving;
  state.blocker  = std::nullopt;
  state.movement = {
      .t = t, .pos = 0, .v = schedule.get_initial_velocity(), .phases = {}};
  if (!m_simulator.get_stop_positions_of_tr(tr).empty()) {
    state.next_stop = 0;
  }
  m_vertex_headways.at(entry) = t + network().get_vertex(entry).headway;
  plan(tr, t);

  // Trains heading onto the same first edge only notice the new train once it
  // is on that edge
  const auto    first_edge = route(tr).front();
  const Blocker on_first_edge{.train = tr, .rear_pos = GRB_EPS - length(tr)};
  for (const auto other : m_trains_on_edges.at(first_edge)) {
    if (other != tr && m_trains.at(other).status == Status::Driving &&
        front(other, t).pos <= m_edge_starts.at(other).at(first_edge)) {
      schedule_at_most(other, release_time(on_first_edge, t));
    }
  }
}

void cda_rail::simulator::EventSimulator::Run::serve_stop(size_t tr,
                                                          size_t stop,
                                                          double t) {
  auto&       state = m_trains.at(tr);
  const auto& scheduled_stop =
      m_instance.get_const_schedule(tr).get_stops().at(stop);
  const auto pos = state.movement.end_pos();
  PLOGV << "At time " << t << ", train " << tr << " stops at "
        << scheduled_stop.get_station().name;
  record(tr, t);
  if (m_save_trajectories) {
    m_trajectories.at(tr)[t] = {.pos = pos, .vel = 0};
  }
  m_stop_times.at(tr).push_back(t);
  state.stop_until = std::max(t + scheduled_stop.get_service_duration(),
                              scheduled_stop.get_earliest_departure());
  state.next_stop  = stop + 1 < m_simulator.get_stop_positions_of_tr(tr).size()
                         ? std::optional{stop + 1}
                         : std::nullopt;
  state.movement   = {.t = t, .pos = pos, .v = 0, .phases = {}};
  state.blocker    = std::nullopt;
  schedule(tr, state.stop_until);
}

void cda_rail::simulator::EventSimulator::Run::leave(size_t tr, double t) {
  auto&       state     = m_trains.at(tr);
  const auto& schedule  = m_instance.get_const_schedule(tr);
  const auto  exit_time = schedule.get_exit_time();
  if (leaves_network(tr) && t < exit_time - GRB_EPS) {
    // A train that could not slow down enough waits until it may leave, as in
    // the greedy simulator
    this->schedule(tr, exit_time);
    return;
  }
  PLOGV << "At time " << t << ", train " << tr << " leaves the network.";
  record(tr, t);
  state.status  = Status::Left;
  state.blocker = std::nullopt;
  if (leaves_network(tr)) {
    // A train slowed down for its exit time reaches it only up to rounding
    const auto left_at     = t < exit_time + GRB_EPS ? exit_time : t;
    const auto exit_vertex = schedule.get_exit_vertex();
    m_exit_times.at(tr)    = left_at;
    m_vertex_headways.at(exit_vertex) =
        left_at + network().get_vertex(exit_vertex).headway;
    if (m_save_trajectories) {
      m_trajectories.at(tr)[left_at] = {.pos = end_pos(tr),
                                        .vel = front(tr, t).vel};
    }
  } else {
    m_exit_times.at(tr) = t;
    if (m_save_trajectories) {
      m_trajectories.at(tr)[t] = {.pos = end_pos(tr), .vel = 0};
    }
    if (m_block_vertices) {
      block_last_vertex(tr, t);
    }
  }
  wake_trains_waiting_for(tr, t);
}

void cda_rail::simulator::EventSimulator::Run::plan(size_t tr, double t) {
  auto&       state = m_trains.at(tr);
  const auto& train = m_instance.get_const_train_list().get_train(tr);
  const auto  now   = state.movement.at(t);
  record(tr, t);
  if (m_save_trajectories) {
    m_trajectories.at(tr)[t] = now;
  }

  auto const target = find_target(tr, t, now);
  state.movement =
      target.end == End::Exit
          ? m_simulator.exit_movement(tr, t, now,
                                      m_limit_speed_by_leaving_edges)
          : Movement{
                .t      = t,
                .pos    = now.pos,
                .v      = now.vel,
                .phases = fastest_phases(
                    now.pos, now.vel, target.pos,
                    target.end == End::RouteEnd ? INF : 0.0,
                    m_simulator.speed_limits(tr, now.pos, target.pos,
                                             m_limit_speed_by_leaving_edges),
                    train.get_acceleration(), train.get_deceleration())};
  state.blocker = std::nullopt;

  auto next = state.movement.end_time();
  if (target.end == End::Obstacle) {
    state.blocker = target.blocker;
    if (target.follows) {
      // The train ahead is treated as if it stood still, which only matters
      // once the train brakes for it
      state.blocker->not_before =
          std::max(t + m_dt, state.movement.final_braking_start());
    }
    next = release_time(*state.blocker, t);
    if (stop_at_end(tr).has_value()) {
      next = std::min(next, state.movement.end_time());
    }
  }
  if (target.checkpoint < INF) {
    next = std::min(
        next, state.movement.braking_point_time(
                  target.checkpoint - (GRB_EPS / 2), train.get_deceleration()));
  }
  if (state.blocked_pos < INF) {
    next = std::min(next,
                    state.movement.time_at(state.blocked_pos + (2 * GRB_EPS)));
  }
  schedule(tr, next);
  wake_trains_waiting_for(tr, t);
}

void cda_rail::simulator::EventSimulator::Run::block_last_vertex(size_t tr,
                                                                 double t) {
  const auto vertex = network().get_edge(route(tr).back()).target;
  for (size_t other = 0; other < m_trains.size(); ++other) {
    auto&      other_state = m_trains.at(other);
    const auto vertex_pos  = route_pos(other, vertex);
    if (other == tr || other_state.status == Status::Left ||
        !vertex_pos.has_value() ||
        (other_state.status == Status::Driving &&
         front(other, t).pos > *vertex_pos)) {
      continue;
    }
    other_state.blocked_pos = std::min(other_state.blocked_pos, *vertex_pos);
    if (other_state.status == Status::Driving) {
      schedule_at_most(other, other_state.movement.time_at(
                                  other_state.blocked_pos + (2 * GRB_EPS)));
    }
  }
}

// Event queue

void cda_rail::simulator::EventSimulator::Run::schedule(size_t tr, double t) {
  auto& state      = m_trains.at(tr);
  state.next_event = t;
  ++state.next_event_id;
  if (t < INF) {
    m_events.emplace(t, tr, state.next_event_id);
  }
}

void cda_rail::simulator::EventSimulator::Run::schedule_at_most(size_t tr,
                                                                double t) {
  if (t < m_trains.at(tr).next_event) {
    schedule(tr, t);
  }
}

void cda_rail::simulator::EventSimulator::Run::wake_trains_waiting_for(
    size_t tr, double t) {
  for (size_t other = 0; other < m_trains.size(); ++other) {
    const auto& blocker = m_trains.at(other).blocker;
    if (blocker.has_value() && blocker->train == tr) {
      schedule_at_most(other, release_time(*blocker, t));
    }
  }
}

bool cda_rail::simulator::EventSimulator::Run::is_released(
    Blocker const& blocker, double t) const {
  if (!blocker.train.has_value()) {
    return t >= blocker.not_before;
  }
  const auto& other = m_trains.at(*blocker.train);
  return other.status == Status::Left ||
         (other.status == Status::Driving &&
          front(*blocker.train, t).pos - length(*blocker.train) >=
              blocker.rear_pos);
}

double
cda_rail::simulator::EventSimulator::Run::release_time(Blocker const& blocker,
                                                       double         t) const {
  if (!blocker.train.has_value()) {
    return std::max(t, blocker.not_before);
  }
  const auto& other = m_trains.at(*blocker.train);
  if (other.status == Status::Left) {
    return t;
  }
  if (other.status == Status::Waiting) {
    return INF;
  }
  // The margin makes sure that the rear has passed despite rounding
  return std::max(
      {t, blocker.not_before,
       other.movement.time_at(blocker.rear_pos + length(*blocker.train) +
                              (GRB_EPS / 2))});
}

// Movement authority, as in GreedySimulator::get_ma_and_maxv

cda_rail::simulator::EventSimulator::Run::Target
cda_rail::simulator::EventSimulator::Run::find_target(size_t tr, double t,
                                                      PosVel now) const {
  const auto& edges = route(tr);
  const auto& m     = m_milestones.at(tr);
  Target      target{.pos = end_pos(tr),
                     .end = leaves_network(tr) ? End::Exit : End::RouteEnd};
  const auto  limit_to = [&target](Target const& other) {
    if (other.pos < target.pos) {
      target = other;
    }
  };

  if (const auto next_stop = m_trains.at(tr).next_stop; next_stop.has_value()) {
    const auto stop_pos =
        m_simulator.get_stop_positions_of_tr(tr).at(*next_stop);
    if (stop_pos <= target.pos) {
      target = {.pos = stop_pos, .end = End::Stop};
    }
  }
  if (const auto blocker = vertex_order_blocker(
          tr, m_instance.get_const_schedule(tr).get_exit_vertex(), t);
      leaves_network(tr) && blocker.has_value()) {
    limit_to({.pos = m.back(), .end = End::Obstacle, .blocker = *blocker});
  }

  const auto braking_point =
      now.pos + m_simulator.tr_braking_distance(tr, now.vel);
  auto checkpoint = INF;
  bool first_edge = true;
  for (size_t i = 0; i < edges.size() && m.at(i) < target.pos; ++i) {
    if (m.at(i + 1) <= now.pos) {
      continue;
    }
    if (now.pos < m.at(i) + GRB_EPS) {
      // The front is about to enter the edge
      if (const auto blocker = ttd_blocker(tr, i, t); blocker.has_value()) {
        limit_to({.pos = m.at(i), .end = End::Obstacle, .blocker = *blocker});
      }
      if (m_block_vertices) {
        const auto reverse = reverse_edge_use(tr, edges.at(i), t);
        if (reverse.blocker.has_value()) {
          limit_to({.pos     = m.at(i),
                    .end     = End::Obstacle,
                    .blocker = *reverse.blocker});
        } else if (reverse.used && m.at(i) > braking_point + GRB_EPS) {
          // Whether the edge is free is only decided once the train cannot
          // stop in front of it anymore
          checkpoint = std::min(checkpoint, m.at(i));
        }
      }
    }
    const auto vertex = network().get_edge(edges.at(i)).target;
    if (m_vertex_headways.at(vertex) > t) {
      limit_to({.pos     = m.at(i + 1),
                .end     = End::Obstacle,
                .blocker = {.not_before = m_vertex_headways.at(vertex)}});
    }
    if (const auto ahead = train_ahead(tr, i, t, now.pos, first_edge);
        ahead.has_value()) {
      limit_to(*ahead);
    }
    first_edge = false;
  }
  target.checkpoint = checkpoint < target.pos ? checkpoint : INF;
  return target;
}

std::optional<cda_rail::simulator::EventSimulator::Run::Target>
cda_rail::simulator::EventSimulator::Run::train_ahead(size_t tr, size_t i,
                                                      double t, double pos,
                                                      bool first_edge) const {
  const auto            edge = route(tr).at(i);
  const auto&           m    = m_milestones.at(tr);
  std::optional<Target> nearest;
  for (const auto other : m_trains_on_edges.at(edge)) {
    const auto position = position_on_edge(other, edge, t);
    if (other == tr || !position.has_value()) {
      continue;
    }
    const auto [rear, front] = *position;
    // On its current edge, a train is not limited by those behind it
    if (first_edge && pos > m.at(i) + EPS &&
        std::max(0.0, pos - length(tr) - m.at(i)) >= front) {
      continue;
    }
    const auto obstacle = m.at(i) + std::max(0.0, rear);
    if (!nearest.has_value() || obstacle < nearest->pos) {
      nearest = Target{.pos     = obstacle,
                       .end     = End::Obstacle,
                       .blocker = {.train    = other,
                                   .rear_pos = this->front(other, t).pos -
                                               length(other) + GRB_EPS},
                       .follows = true};
    }
  }
  return nearest;
}

std::optional<cda_rail::simulator::EventSimulator::Run::Blocker>
cda_rail::simulator::EventSimulator::Run::entry_blocker(size_t tr,
                                                        double t) const {
  // As in GreedySimulator::get_entering_trains and is_ok_to_enter
  const auto& schedule = m_instance.get_const_schedule(tr);
  const auto  entry    = schedule.get_entry_vertex();
  if (const auto blocker = vertex_order_blocker(tr, entry, t);
      blocker.has_value()) {
    return blocker;
  }
  if (m_vertex_headways.at(entry) > t) {
    return Blocker{.not_before = m_vertex_headways.at(entry)};
  }

  // The train's braking distance has to be free
  const auto braking_distance =
      m_simulator.tr_braking_distance(tr, schedule.get_initial_velocity());
  const auto& edges = route(tr);
  const auto& m     = m_milestones.at(tr);
  for (size_t i = 0; i < edges.size() && m.at(i) + EPS < braking_distance;
       ++i) {
    const auto free        = braking_distance - m.at(i);
    const auto edge_length = network().get_edge(edges.at(i)).length;
    for (const auto other : m_trains_on_edges.at(edges.at(i))) {
      const auto position = position_on_edge(other, edges.at(i), t);
      if (other != tr && position.has_value() &&
          std::max(0.0, position->first) + EPS <= free) {
        return Blocker{.train    = other,
                       .rear_pos = m_edge_starts.at(other).at(edges.at(i)) +
                                   std::min(free, edge_length) + GRB_EPS};
      }
    }
    if (const auto blocker = reverse_edge_use(tr, edges.at(i), t).blocker;
        blocker.has_value()) {
      return blocker;
    }
    if (const auto blocker = ttd_blocker(tr, i, t); blocker.has_value()) {
      return blocker;
    }
  }
  return std::nullopt;
}

std::optional<cda_rail::simulator::EventSimulator::Run::Blocker>
cda_rail::simulator::EventSimulator::Run::ttd_blocker(size_t tr, size_t i,
                                                      double t) const {
  const auto& edges = route(tr);
  const auto  ttd   = m_simulator.get_ttd(edges.at(i));
  // Within a TTD section, this has been checked when entering it
  if (!ttd.has_value() ||
      (i > 0 && m_simulator.get_ttd(edges.at(i - 1)) == ttd)) {
    return std::nullopt;
  }
  const auto& order = m_simulator.get_ttd_orders_of_ttd(*ttd);
  const auto  it    = std::ranges::find(order, tr);
  if (it == order.begin()) {
    return std::nullopt;
  }
  const auto    previous = *std::prev(it);
  const Blocker blocker{.train = previous, .rear_pos = ttd_end(previous, *ttd)};
  return is_released(blocker, t) ? std::nullopt : std::optional{blocker};
}

cda_rail::simulator::EventSimulator::Run::ReverseEdgeUse
cda_rail::simulator::EventSimulator::Run::reverse_edge_use(size_t tr,
                                                           size_t edge,
                                                           double t) const {
  // A train on the reverse edge occupies it up to its braking point, as in
  // GreedySimulator::is_ok_to_enter, and a train that has not passed it yet
  // may still use it
  ReverseEdgeUse use{};
  const auto     reverse = network().get_reverse_edge_index(edge);
  if (!reverse.has_value()) {
    return use;
  }
  const auto reverse_length = network().get_edge(*reverse).length;
  for (const auto other : m_trains_on_edges.at(*reverse)) {
    const auto status = m_trains.at(other).status;
    if (other == tr || status == Status::Left) {
      continue;
    }
    if (status == Status::Waiting) {
      use.used = true;
      continue;
    }
    const auto [pos, vel] = front(other, t);
    const auto start      = m_edge_starts.at(other).at(*reverse);
    if (pos - length(other) >= start + reverse_length - EPS) {
      continue; // The other train has passed the reverse edge
    }
    use.used = true;
    if (pos + m_simulator.tr_braking_distance(other, vel) > start + EPS) {
      use.blocker = Blocker{.train = other, .rear_pos = start + reverse_length};
      return use;
    }
  }
  return use;
}

std::optional<cda_rail::simulator::EventSimulator::Run::Blocker>
cda_rail::simulator::EventSimulator::Run::vertex_order_blocker(size_t tr,
                                                               size_t vertex,
                                                               double t) const {
  // The train before in the vertex's order has to have entered the network
  // there or to have left it, as in GreedySimulator::get_entering_trains and
  // GreedySimulator::is_exit_vertex_blocked
  const auto& order = m_simulator.get_vertex_orders_of_vertex(vertex);
  const auto  it    = std::ranges::find(order, tr);
  if (it == order.begin() || it == order.end()) {
    return std::nullopt;
  }
  const auto    previous = *std::prev(it);
  const Blocker blocker{
      .train = previous,
      .rear_pos =
          m_instance.get_const_schedule(previous).get_entry_vertex() == vertex
              ? -INF
              : INF};
  return is_released(blocker, t) ? std::nullopt : std::optional{blocker};
}

// Positions

std::optional<std::pair<double, double>>
cda_rail::simulator::EventSimulator::Run::position_on_edge(size_t tr,
                                                           size_t edge,
                                                           double t) const {
  // The rear and front relative to the edge's start, if the train is on it,
  // as in GreedySimulator::get_position_on_edge
  if (m_trains.at(tr).status != Status::Driving) {
    return std::nullopt;
  }
  const auto front_on_edge = front(tr, t).pos - m_edge_starts.at(tr).at(edge);
  const auto rear_on_edge  = front_on_edge - length(tr);
  if (front_on_edge <= EPS ||
      rear_on_edge >= network().get_edge(edge).length - EPS) {
    return std::nullopt;
  }
  return std::pair{rear_on_edge,
                   std::min(front_on_edge, network().get_edge(edge).length)};
}

double cda_rail::simulator::EventSimulator::Run::ttd_end(size_t tr,
                                                         size_t ttd) const {
  // Where a train's rear has left a TTD section, INF if it never enters it
  const auto& section = m_simulator.get_ttd_sections().at(ttd);
  const auto& edges   = route(tr);
  auto        end     = INF;
  for (size_t i = 0; i < edges.size(); ++i) {
    if (section.contains(edges.at(i))) {
      end = m_milestones.at(tr).at(i + 1);
    }
  }
  return end;
}

std::optional<double>
cda_rail::simulator::EventSimulator::Run::route_pos(size_t tr,
                                                    size_t vertex) const {
  // As in GreedySimulator::get_vertex_pos
  const auto& edges = route(tr);
  for (size_t i = 0; i < edges.size(); ++i) {
    const auto& edge = network().get_edge(edges.at(i));
    if (i == 0 && edge.source == vertex) {
      return 0.0;
    }
    if (edge.target == vertex) {
      return m_milestones.at(tr).at(i + 1);
    }
  }
  return std::nullopt;
}

bool cda_rail::simulator::EventSimulator::Run::leaves_network(size_t tr) const {
  return network().get_edge(route(tr).back()).target ==
         m_instance.get_const_schedule(tr).get_exit_vertex();
}

double cda_rail::simulator::EventSimulator::Run::end_pos(size_t tr) const {
  // A train leaves once its rear has passed its exit vertex
  return m_milestones.at(tr).back() + (leaves_network(tr) ? length(tr) : 0.0);
}

std::optional<size_t>
cda_rail::simulator::EventSimulator::Run::stop_at_end(size_t tr) const {
  // The next stop if the train's movement ends with it standing close enough
  // to serve it, as in the greedy simulator
  const auto& state     = m_trains.at(tr);
  const auto  next_stop = state.next_stop;
  if (next_stop.has_value() && state.movement.end_speed() < V_MIN &&
      state.movement.end_pos() >=
          m_simulator.get_stop_positions_of_tr(tr).at(*next_stop) -
              STOP_TOLERANCE) {
    return next_stop;
  }
  return std::nullopt;
}

// Trajectories

void cda_rail::simulator::EventSimulator::Run::record(size_t tr, double t) {
  // Saves where the phases of the train's movement ended before t and where
  // its front passed a vertex, as the moving block MIP does
  if (!m_save_trajectories) {
    return;
  }
  const auto&         movement   = m_trains.at(tr).movement;
  auto&               trajectory = m_trajectories.at(tr);
  std::vector<double> times;
  double              time = movement.t;
  for (const auto& phase : movement.phases) {
    time += phase.duration;
    times.push_back(time);
  }
  for (const auto vertex_pos : m_milestones.at(tr)) {
    times.push_back(movement.time_at(vertex_pos));
  }
  for (const auto sample_time : times) {
    if (movement.t < sample_time && sample_time < t - GRB_EPS) {
      trajectory[sample_time] = movement.at(sample_time);
    }
  }
}

void cda_rail::simulator::EventSimulator::Run::remove_uninformative_samples(
    size_t tr) {
  // Only the samples at vertices and those at which the acceleration changes
  // are needed to describe a trajectory
  auto& trajectory = m_trajectories.at(tr);
  if (trajectory.size() < 3) {
    return;
  }
  const auto acceleration = [](auto from, auto to) {
    return (to->second.vel - from->second.vel) / (to->first - from->first);
  };
  const auto at_vertex = [this, tr](double pos) {
    return std::ranges::any_of(m_milestones.at(tr), [pos](double vertex_pos) {
      return std::abs(vertex_pos - pos) < GRB_EPS;
    });
  };
  auto previous = trajectory.begin();
  auto it       = std::next(previous);
  while (std::next(it) != trajectory.end()) {
    const auto next = std::next(it);
    const auto distance_matches =
        std::abs(next->second.pos - previous->second.pos -
                 ((previous->second.vel + next->second.vel) *
                  (next->first - previous->first) / 2)) < GRB_EPS;
    if (!at_vertex(it->second.pos) && distance_matches &&
        std::abs(acceleration(previous, it) - acceleration(it, next)) <
            GRB_EPS) {
      it = trajectory.erase(it);
    } else {
      previous = it;
      it       = next;
    }
  }
}

// ---------------
// SIMULATE

cda_rail::simulator::SimulatorResults
cda_rail::simulator::EventSimulator::simulate(
    double dt, bool late_entry_possible, bool limit_speed_by_leaving_edges,
    bool save_trajectories, bool block_vertices_after_disappearing) const {
  exceptions::throw_if_non_positive(dt, "Time step length dt");
  cda_rail::initialize_plog(false);
  Run run(*this, dt, late_entry_possible, limit_speed_by_leaving_edges,
          save_trajectories, block_vertices_after_disappearing);
  return run.simulate();
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
