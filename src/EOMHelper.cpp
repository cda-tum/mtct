#include "EOMHelper.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace cda_rail {
namespace { // anonymous namespace for internal linkage

template <typename... Args> void round_with_default_eps(Args&... args) {
  // Refactoring helper: normalize multiple values with default tolerance.
  (round_small_numbers_to_zero_inplace(args), ...);
}

template <typename... Args> void round_with_eps(double eps, Args&... args) {
  // Refactoring helper: normalize multiple values with an explicit tolerance.
  (round_small_numbers_to_zero_inplace(args, eps), ...);
}

[[nodiscard]] double square(double value) { return value * value; }

[[nodiscard]] double braking_distance_unchecked(double v, double d) {
  return square(v) / (2 * d);
}

[[nodiscard]] double stable_ratio_with_sqrt(double numerator, double scale,
                                            double radicand,
                                            double denominator_offset) {
  // Shared stable form: numerator / (scale * (sqrt(radicand) + offset)).
  return numerator / (scale * (std::sqrt(radicand) + denominator_offset));
}

[[nodiscard]] double stable_phase_time(double distance, double initial_speed,
                                       double acceleration) {
  // Returns the time to cover `distance` from `initial_speed` at constant
  // `acceleration`. Original: t = (sqrt(v0^2 + 2*a*s) - v0) / a Stable:   t =
  // 2*s / (sqrt(v0^2 + 2*a*s) + v0) Both are algebraically equivalent; the
  // stable form avoids cancellation.
  if (distance == 0) {
    return 0;
  }

  const double radicand =
      std::max(0.0, square(initial_speed) + 2 * acceleration * distance);
  return stable_ratio_with_sqrt(2 * distance, 1.0, radicand, initial_speed);
}

struct DistancePhases {
  double first;
  double second;
  double third;
};

[[nodiscard]] DistancePhases
split_distance_by_change_points(double x, double s_1, double s_2, double s) {
  // Refactoring helper: split traveled distance into the 3 profile phases.
  return {
      std::min(x, s_1),
      std::clamp(x - s_1, 0.0, s_2 - s_1),
      std::clamp(x - s_2, 0.0, s - s_2),
  };
}

void normalize_ma_push_inputs(double& v_0, double& a, double& d, double& s) {
  // Refactoring helper: centralize MA-input rounding and near-zero handling.
  round_with_eps(GRB_EPS, v_0, a, s);
  if (d < 0 && d > -GRB_EPS) {
    d = 0;
  }
}

[[nodiscard]] double normalize_obd(double obd) {
  // Refactoring helper: shared validation for overlap braking distance.
  round_with_eps(GRB_EPS, obd);
  exceptions::throw_if_negative(obd, "obd");
  return obd;
}

struct LineSpeedProfile {
  double v_1;
  double v_2;
  double v_line;
  double s;
  double total_time;
  double a_1;
  double a_2;
  double t_1;
  double t_2;
  double s_1;
};

[[nodiscard]] LineSpeedProfile build_line_speed_profile(double v_1, double v_2,
                                                        double v_line, double a,
                                                        double d, double s) {
  // Refactoring helper: compute common phase data once for time/pos/vel calls.
  round_with_eps(GRB_EPS, v_1, v_2, v_line, a, d, s);
  exceptions::throw_if_negative(v_1, "v_1");
  exceptions::throw_if_negative(v_2, "v_2");
  exceptions::throw_if_non_positive(v_line, GRB_EPS, "v_line");
  exceptions::throw_if_non_positive(a, GRB_EPS, "a");
  exceptions::throw_if_non_positive(d, GRB_EPS, "d");
  exceptions::throw_if_negative(s, "s");

  const double a_1 = v_line >= v_1 ? a : -d;
  const double s_1 = (square(v_line) - square(v_1)) / (2 * a_1);
  const double t_1 = (v_line - v_1) / a_1;
  assert(t_1 >= 0);

  const double a_2        = v_2 >= v_line ? a : -d;
  const double s_2        = (square(v_2) - square(v_line)) / (2 * a_2);
  const double t_2_change = (v_2 - v_line) / a_2;
  assert(t_2_change >= 0);

  if (s_1 + s_2 - GRB_EPS > s) {
    throw exceptions::ConsistencyException(
        "Travel time not possible by equations of motion.");
  }

  const double cruise_time =
      std::abs(s - s_1 - s_2) < GRB_EPS ? 0 : (s - s_1 - s_2) / v_line;
  const double total_time = t_1 + t_2_change + cruise_time;

  return {
      v_1, v_2, v_line, s, total_time, a_1, a_2, t_1, total_time - t_2_change,
      s_1};
}

[[nodiscard]] bool time_is_after_profile_end(double v_1, double v_2,
                                             double v_line, double a, double d,
                                             double s, double t,
                                             double      total_time,
                                             const char* function_name) {
  // Refactoring helper: shared tolerant end-of-profile boundary handling.
  if (t <= total_time + GRB_EPS) {
    return false;
  }

  const double v_line_dv = v_line - LINE_SPEED_ACCURACY;
  const double total_time_dv =
      cda_rail::time_on_edge(v_1, v_2, v_line_dv, a, d, s);
  if (t > total_time_dv + GRB_EPS) {
    throw exceptions::InvalidInputException(
        "Time exceeds total travel time in cda_rail::" +
        std::string(function_name) + " with t = " + std::to_string(t) +
        " and total_time_dv = " + std::to_string(total_time_dv));
  }

  return true;
}

} // namespace
} // namespace cda_rail

// ---------------------------
// GENERAL MEASURES
// ---------------------------

double cda_rail::braking_distance(double v, double d) {
  exceptions::throw_if_negative(v, "Speed");
  exceptions::throw_if_non_positive(d, EPS, "Deceleration");

  return braking_distance_unchecked(v, d);
}

double cda_rail::max_braking_pos_after_dt_linear_movement(double v_0,
                                                          double v_max,
                                                          double a, double d,
                                                          int dt) {
  round_with_default_eps(v_0, v_max, a, d);
  if (std::abs(v_0 - v_max) < EPS) {
    v_max = v_0;
  }

  exceptions::throw_if_less_than(static_cast<double>(dt), 0.0, "Time");
  exceptions::throw_if_negative(v_0, "Initial speed");
  if (v_max < v_0) {
    throw exceptions::InvalidInputException(
        "Maximum speed must be greater than or equal to initial speed.");
  }
  exceptions::throw_if_non_positive(a, "Acceleration");
  exceptions::throw_if_non_positive(d, "Deceleration");

  const double reached_speed      = std::min(v_max, v_0 + a * dt);
  const double travelled_distance = (v_0 + reached_speed) * dt / 2;
  return travelled_distance + braking_distance(reached_speed, d);
}

// ---------------------------
// MINIMAL TRAVEL TIMES
// ---------------------------

double cda_rail::min_travel_time(double v_1, double v_2, double v_m, double a,
                                 double d, double s) {
  return min_travel_time_from_start(v_1, v_2, v_m, a, d, s, s);
}

double cda_rail::min_travel_time_from_start(double v_1, double v_2, double v_m,
                                            double a, double d, double s,
                                            double x) {
  check_consistency_of_eom_input(v_1, v_2, a, d, s, x);

  const auto [s_1, s_2] =
      get_min_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d, s);

  assert(s_2 >= s_1);

  const auto [x_1, x_2, x_3] = split_distance_by_change_points(x, s_1, s_2, s);

  const double v_t_squared = square(v_1) + 2 * a * s_1;
  const double v_t         = std::sqrt(v_t_squared);

  return stable_phase_time(x_1, v_1, a) + (x_2 == 0 ? 0 : x_2 / v_t) +
         stable_phase_time(x_3, v_t, -d);
}

double cda_rail::min_travel_time_to_end(double v_1, double v_2, double v_m,
                                        double a, double d, double s,
                                        double x) {
  // Same as minimal travel time going the reverse direction with acceleration
  // and deceleration swapped
  // NOLINTNEXTLINE(readability-suspicious-call-argument)
  return min_travel_time_from_start(v_2, v_1, v_m, d, a, s, s - x);
}

// ----------------------------
// MAX TRAVEL TIMES
// ----------------------------

double cda_rail::max_travel_time(double v_1, double v_2, double v_m, double a,
                                 double d, double s, bool stopping_allowed) {
  return max_travel_time_from_start(v_1, v_2, v_m, a, d, s, s,
                                    stopping_allowed);
}

double cda_rail::max_travel_time_no_stopping(double v_1, double v_2, double v_m,
                                             double a, double d, double s) {
  return max_travel_time_from_start_no_stopping(v_1, v_2, v_m, a, d, s, s);
}

double cda_rail::max_travel_time_stopping_allowed(double v_1, double v_2,
                                                  double a, double d,
                                                  double s) {
  return max_travel_time_from_start_stopping_allowed(v_1, v_2, a, d, s, s);
}

double cda_rail::max_travel_time_from_start(double v_1, double v_2, double v_m,
                                            double a, double d, double s,
                                            double x, bool stopping_allowed) {
  return stopping_allowed
             ? max_travel_time_from_start_stopping_allowed(v_1, v_2, a, d, s, x)
             : max_travel_time_from_start_no_stopping(v_1, v_2, v_m, a, d, s,
                                                      x);
}

double cda_rail::max_travel_time_from_start_no_stopping(double v_1, double v_2,
                                                        double v_m, double a,
                                                        double d, double s,
                                                        double x) {
  // v_m is minimal speed in this case

  check_consistency_of_eom_input(v_1, v_2, a, d, s, x);

  const bool v_1_below_minimal_speed = v_1 < v_m;
  const bool v_2_below_minimal_speed = v_2 < v_m;

  const auto [s_1, s_2] =
      get_max_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d, s);
  const auto [x_1, x_2, x_3] = split_distance_by_change_points(x, s_1, s_2, s);

  const double a_1         = v_1_below_minimal_speed ? a : -d;
  const double a_3         = v_2_below_minimal_speed ? -d : a;
  const double v_t_squared = square(v_1) + 2 * a_1 * s_1;
  const double v_t         = std::sqrt(v_t_squared);

  return stable_phase_time(x_1, v_1, a_1) + (x_2 == 0 ? 0 : x_2 / v_t) +
         stable_phase_time(x_3, v_t, a_3);
}

double cda_rail::max_travel_time_from_start_stopping_allowed(
    double v_1, double v_2, double a, double d, double s, double x) {
  check_consistency_of_eom_input(v_1, v_2, a, d, s, x);

  const double s_1 =
      get_max_travel_time_acceleration_change_points(v_1, v_2, 0, a, d, s)
          .first;
  const double bd = braking_distance_unchecked(v_1, d);

  if (bd <= s_1 + EPS && x + EPS >= s_1) {
    return std::numeric_limits<double>::infinity();
  }

  // Train cannot stop, hence same as max_travel_time_from_start_no_stopping
  return max_travel_time_from_start_no_stopping(v_1, v_2, 0, a, d, s, x);
}

double cda_rail::max_travel_time_to_end(double v_1, double v_2, double v_m,
                                        double a, double d, double s, double x,
                                        bool stopping_allowed) {
  return stopping_allowed
             ? max_travel_time_to_end_stopping_allowed(v_1, v_2, a, d, s, x)
             : max_travel_time_to_end_no_stopping(v_1, v_2, v_m, a, d, s, x);
}

double cda_rail::max_travel_time_to_end_no_stopping(double v_1, double v_2,
                                                    double v_m, double a,
                                                    double d, double s,
                                                    double x) {
  // Same as maximal travel time going the reverse direction with acceleration
  // and deceleration swapped
  // NOLINTNEXTLINE(readability-suspicious-call-argument)
  return max_travel_time_from_start_no_stopping(v_2, v_1, v_m, d, a, s, s - x);
}

double cda_rail::max_travel_time_to_end_stopping_allowed(double v_1, double v_2,
                                                         double a, double d,
                                                         double s, double x) {
  // Same as maximal travel time going the reverse direction with acceleration
  // and deceleration swapped
  // NOLINTNEXTLINE(readability-suspicious-call-argument)
  return max_travel_time_from_start_stopping_allowed(v_2, v_1, d, a, s, s - x);
}

// ---------------------------
// LINE SPEED CALCULATIONS
// ---------------------------

double cda_rail::minimal_line_speed(double v_1, double v_2, double v_min,
                                    double a, double d, double s) {
  const bool v_1_below_minimal_speed = v_1 < v_min;

  const auto [s_1, s_2] =
      get_max_travel_time_acceleration_change_points(v_1, v_2, v_min, a, d, s);

  (void)s_2; // unused

  const double a_1 = v_1_below_minimal_speed ? a : -d;
  return std::sqrt(square(v_1) + 2 * a_1 * s_1);
}

double cda_rail::maximal_line_speed(double v_1, double v_2, double v_max,
                                    double a, double d, double s) {
  const auto [s_1, s_2] =
      get_min_travel_time_acceleration_change_points(v_1, v_2, v_max, a, d, s);

  assert(s_2 >= s_1);

  return std::sqrt(square(v_1) + 2 * a * s_1);
}

double cda_rail::get_line_speed(double v_1, double v_2, double v_min,
                                double v_max, double a, double d, double s,
                                double t) {
  if (max_travel_time_no_stopping(v_1, v_2, v_min, a, d, s) < t - GRB_EPS) {
    return 0;
  }

  double v_ub = maximal_line_speed(v_1, v_2, v_max, a, d, s);
  double v_lb = minimal_line_speed(v_1, v_2, v_min, a, d, s);

  const double t_ub = time_on_edge(v_1, v_2, v_lb, a, d, s);
  double       t_lb = time_on_edge(v_1, v_2, v_ub, a, d, s);

  if (std::abs(t_lb - t) < GRB_EPS) {
    return v_ub;
  }
  if (std::abs(t_ub - t) < GRB_EPS) {
    return v_lb;
  }

  assert(t_lb < t);
  assert(t < t_ub);
  while (v_ub - v_lb > LINE_SPEED_ACCURACY &&
         t - t_lb > LINE_SPEED_TIME_ACCURACY) {
    const double v = (v_ub + v_lb) / 2;
    if (const double t_v = time_on_edge(v_1, v_2, v, a, d, s); t_v <= t) {
      v_ub = v;
      t_lb = t_v;
    } else {
      v_lb = v;
    }
  }

  return v_ub;
}

double cda_rail::time_on_edge(double v_1, double v_2, double v_line, double a,
                              double d, double s) {
  /**
   * This function calculates the time a train needs to travel distance s with
   * - initial speed v_1
   * - final speed v_2
   * - line speed v_line
   * - acceleration a
   * - deceleration d
   */

  return build_line_speed_profile(v_1, v_2, v_line, a, d, s).total_time;
}

double cda_rail::pos_on_edge_at_time(double v_1, double v_2, double v_line,
                                     double a, double d, double s, double t) {
  const auto profile    = build_line_speed_profile(v_1, v_2, v_line, a, d, s);
  const auto total_time = profile.total_time;
  if (std::abs(total_time) < GRB_EPS) {
    return 0;
  }
  if (total_time < 0) {
    throw exceptions::InvalidInputException("Total travel time is negative.");
  }
  if (time_is_after_profile_end(v_1, v_2, v_line, a, d, s, t, total_time,
                                "pos_on_edge_at_time")) {
    return profile.s;
  }
  if (std::abs(t - total_time) < GRB_EPS) {
    return profile.s;
  }

  if (t <= profile.t_1) {
    return (profile.v_1 * t) + (0.5 * profile.a_1 * t * t);
  }
  if (t <= profile.t_2) {
    return profile.s_1 + (profile.v_line * (t - profile.t_1));
  }

  const double remaining_time = total_time - t;
  return profile.s + (0.5 * profile.a_2 * remaining_time * remaining_time) -
         (profile.v_2 * remaining_time);
}

double cda_rail::vel_on_edge_at_time(double v_1, double v_2, double v_line,
                                     double a, double d, double s, double t) {
  const auto profile    = build_line_speed_profile(v_1, v_2, v_line, a, d, s);
  const auto total_time = profile.total_time;
  if (std::abs(total_time) < GRB_EPS) {
    return profile.v_1;
  }
  if (total_time < 0) {
    throw exceptions::InvalidInputException("Total travel time is negative.");
  }
  if (time_is_after_profile_end(v_1, v_2, v_line, a, d, s, t, total_time,
                                "vel_on_edge_at_time")) {
    return profile.v_2;
  }
  if (std::abs(t - total_time) < GRB_EPS) {
    return profile.v_2;
  }

  if (t <= profile.t_1) {
    return profile.v_1 + (profile.a_1 * t);
  }
  if (t <= profile.t_2) {
    return profile.v_line;
  }

  return profile.v_2 - (profile.a_2 * (total_time - t));
}

// ---------------------------
// MOVING AUTHORITY CALCULATIONS
// ---------------------------

double cda_rail::min_time_to_push_ma_forward(double v_0, double a, double d,
                                             double s) {
  // How much time does a train need to move its moving authority forward by s
  // given initial speed v_0 and acceleration a and deceleration d

  normalize_ma_push_inputs(v_0, a, d, s);

  exceptions::throw_if_negative(v_0, "v_0");
  exceptions::throw_if_negative(a, "a");
  exceptions::throw_if_non_positive(d, "d");
  exceptions::throw_if_negative(s, "s");

  // ma(t) = v_0*t + 0.5*a*t^2 + (v_0+a*t)^2/(2d) != s + v_0^2/(2d)
  // -> t = (...) = 2*d*s / (sqrt(2*(a+d)*a*d*s+(a+d)^2*v^2) + (a+d)*v)
  if (s == 0) {
    return 0;
  }

  const double a_plus_d = a + d;
  return stable_ratio_with_sqrt(
      2 * d * s, 1.0, 2 * a_plus_d * a * d * s + square(a_plus_d * v_0),
      a_plus_d * v_0);
}

double cda_rail::min_time_to_push_ma_backward(double v_0, double a, double d,
                                              double s) {
  normalize_ma_push_inputs(v_0, a, d, s);

  exceptions::throw_if_negative(v_0, "v_0");
  exceptions::throw_if_negative(a, "a");
  exceptions::throw_if_non_positive(d, "d");
  exceptions::throw_if_negative(s, "s");

  // Assert that s <= v_0^2/(2d)
  const double braking_distance = braking_distance_unchecked(v_0, d);
  if (s > braking_distance + GRB_EPS) {
    throw exceptions::InvalidInputException(
        "s must be less than or equal to v_0^2/(2d)");
  }

  if (std::abs(braking_distance - s) < GRB_EPS) {
    // More stable version in this special case
    return min_time_to_push_ma_fully_backward(v_0, a, d);
  }

  // Solve (v_0-a*t)^2/(2d) + s = v_0*t - 0.5*a*t + v_0^2/(2d)
  // WolframAlpha: solve for t: (v-a*t)^2/(2*b)+s=v*t-0.5*a*t^2+v^2/(2*b), v>0,
  // t>0, a>0, b>0, s>0, s<v^2/(2*b), t<v/a
  // -> t = v/a - sqrt((b v^2 + a (-2 b s + v^2))/(a^2 (a + b)))
  // -> t = (v - sqrt(-(2 a b s)/(a + b) + v^2))/a
  // Numerical stable version: t = 2*b*s/((a+b)*(v+sqrt(v^2-2*a*b*s/(a+b))))
  // Return 0 if v=0 and s = 0
  // using b=d in wolframalpha to prevent conversion to day

  if (v_0 == 0 || s == 0) {
    return 0;
  }

  const double a_plus_d = a + d;
  return stable_ratio_with_sqrt(2 * d * s, a_plus_d,
                                square(v_0) - 2 * a * d * s / a_plus_d, v_0);
}

double cda_rail::min_time_to_push_ma_fully_backward(double v_0, double a,
                                                    double d) {
  // Simplified version if s is the full braking distance
  // solve for t : (v-a*t)^2/(2*b) = v * t - 0.5 * a * t^2
  // -> t = (v + (a v)/b - (sqrt(a + b) v)/sqrt(b))/(a + a^2/b)
  // -> t = ((-sqrt(b) + sqrt(a + b)) v)/(a sqrt(a + b))
  // -> t = v / (a + b + sqrt(b*(a+b)))
  // Again b is replaced with the deceleration

  normalize_ma_push_inputs(v_0, a, d, v_0);

  exceptions::throw_if_negative(v_0, "v_0");
  exceptions::throw_if_negative(a, "a");
  exceptions::throw_if_non_positive(d, "d");

  return v_0 / (a + d + std::sqrt(d * (a + d)));
}

double cda_rail::min_time_from_front_to_ma_point(double v_1, double v_2,
                                                 double v_m, double a, double d,
                                                 double s, double obd) {
  const auto [s_1, s_2] =
      get_min_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d, s);

  obd = normalize_obd(obd);
  if (obd == 0) {
    return min_travel_time(v_1, v_2, v_m, a, d, s);
  }

  const double bd_1  = braking_distance_unchecked(v_1, d);
  const double bd_2  = braking_distance_unchecked(v_2, d);
  const double ubd_1 = s + bd_2 - obd - bd_1;

  if (ubd_1 < -GRB_EPS) {
    throw exceptions::ConsistencyException(
        "obd is too large for the given edge and speed profile.");
  }

  // Is the relevant point within the constant phase or acceleration (Note:
  // deceleration does not change ma)
  if (s_2 - s_1 >= obd) {
    // Point is at s_2 - obd
    return min_travel_time_from_start(v_1, v_2, v_m, a, d, s, s_2 - obd);
  }
  return min_time_to_push_ma_forward(v_1, a, d, ubd_1);
}

double cda_rail::max_time_from_front_to_ma_point(double v_1, double v_2,
                                                 double v_m, double a, double d,
                                                 double s, double obd,
                                                 bool stopping_allowed) {
  return stopping_allowed
             ? max_time_from_front_to_ma_point_stopping_allowed(v_1, v_2, a, d,
                                                                s, obd)
             : max_time_from_front_to_ma_point_no_stopping(v_1, v_2, v_m, a, d,
                                                           s, obd);
}

double cda_rail::max_time_from_front_to_ma_point_no_stopping(
    double v_1, double v_2, double v_m, double a, double d, double s,
    double obd) {
  const auto [s_1, s_2] =
      get_max_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d, s);

  obd = normalize_obd(obd);

  const bool   v1_below_minimal_speed = v_1 < v_m;
  const double a_1                    = v1_below_minimal_speed ? a : -d;

  const double v_t_squared = square(v_1) + (2 * a_1 * s_1);
  const double v_t         = std::sqrt(v_t_squared);

  const double bd_1 = braking_distance_unchecked(v_1, d);
  const double bd_2 = braking_distance_unchecked(v_2, d);
  const double bd_t = braking_distance_unchecked(v_t, d);

  const double ma_point = s + bd_2 - obd;

  const double ubd_v1 = ma_point - bd_1;
  const double ubd_s1 = ma_point - (s_1 + bd_t);
  const double ubd_s2 = ma_point - (s_2 + bd_t);

  if (ubd_s2 > GRB_EPS) {
    return max_travel_time_from_start_no_stopping(v_1, v_2, v_m, a, d, s, s_2) +
           min_time_to_push_ma_forward(v_t, a, d, ubd_s2);
  }
  if (ubd_s1 > GRB_EPS) {
    return max_travel_time_from_start_no_stopping(v_1, v_2, v_m, a, d, s,
                                                  s_1 + ubd_s1);
  }

  if (!v1_below_minimal_speed && std::abs(ubd_s1) <= GRB_EPS) {
    return 0;
  }
  assert(v1_below_minimal_speed);

  return min_time_to_push_ma_forward(v_1, a, d, ubd_v1);
}

double cda_rail::max_time_from_front_to_ma_point_stopping_allowed(
    double v_1, double v_2, double a, double d, double s, double obd) {
  if (max_travel_time_stopping_allowed(v_1, v_2, a, d, s) >=
      std::numeric_limits<double>::infinity()) {
    return std::numeric_limits<double>::infinity();
  }
  return max_time_from_front_to_ma_point_no_stopping(v_1, v_2, 0, a, d, s, obd);
}

double cda_rail::min_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd, cda_rail::MATimingStrategy strategy) {
  if (strategy != MATimingStrategy::ExtremeProfiles) {
    throw exceptions::InvalidInputException("Invalid strategy.");
  }

  const double t1 =
      max_time_profile_from_rear_to_ma_point(v_1, v_2, v_min, a, d, s, obd);
  const double t2 =
      min_time_profile_from_rear_to_ma_point(v_1, v_2, v_max, a, d, s, obd);
  return std::min(t1, t2);
}

double cda_rail::max_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd, cda_rail::MATimingStrategy strategy) {
  if (strategy != MATimingStrategy::ExtremeProfiles) {
    throw exceptions::InvalidInputException("Invalid strategy.");
  }

  const double t1 =
      max_time_profile_from_rear_to_ma_point(v_1, v_2, v_min, a, d, s, obd);
  const double t2 =
      min_time_profile_from_rear_to_ma_point(v_1, v_2, v_max, a, d, s, obd);
  return std::max(t1, t2);
}

double cda_rail::min_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                                        double v_m, double a,
                                                        double d, double s,
                                                        double obd) {
  return min_travel_time(v_1, v_2, v_m, a, d, s) -
         min_time_from_front_to_ma_point(v_1, v_2, v_m, a, d, s, obd);
}

double cda_rail::max_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                                        double v_m, double a,
                                                        double d, double s,
                                                        double obd) {
  return max_travel_time_no_stopping(v_1, v_2, v_m, a, d, s) -
         max_time_from_front_to_ma_point_no_stopping(v_1, v_2, v_m, a, d, s,
                                                     obd);
}

// ---------------------------
// HELPER
// ---------------------------

void cda_rail::check_consistency_of_eom_input(double& v_1, double& v_2,
                                              double& a, double& d, double& s,
                                              double& x) {
  round_with_eps(GRB_EPS, v_1, v_2, a, d, s, x);
  if (v_1 < 0 || v_2 < 0 || a < 0 || d < 0 || s < 0 || x < 0) {
    throw exceptions::ConsistencyException(
        "All input values must be non-negative.");
  }
  if (a < GRB_EPS || d < GRB_EPS) {
    throw exceptions::ConsistencyException(
        "Both acceleration and deceleration must be strictly positive.");
  }

  if (x > s) {
    throw exceptions::ConsistencyException(
        "x must be less than or equal to s.");
  }

  if (!possible_by_eom(v_1, v_2, a, d, s)) {
    throw exceptions::ConsistencyException(
        "Entry and exit velocities not possible by equations of motion.");
  }
}

bool cda_rail::possible_by_eom(double v_1, double v_2, double a, double d,
                               double s) {
  return v_1 <= v_2 ? square(v_2) - square(v_1) <= 2 * a * s + GRB_EPS
                    : square(v_1) - square(v_2) <= 2 * d * s + GRB_EPS;
}

std::pair<double, double>
cda_rail::get_min_travel_time_acceleration_change_points(double v_1, double v_2,
                                                         double v_m, double a,
                                                         double d, double s) {
  check_consistency_of_eom_input(v_1, v_2, a, d, s, s);
  if (v_m <= 0) {
    throw exceptions::ConsistencyException("v_m must be greater than 0.");
  }
  if (v_1 > v_m || v_2 > v_m) {
    throw exceptions::ConsistencyException(
        "v_m must be greater than or equal to v_1 and v_2.");
  }

  const double s_1 = (square(v_m) - square(v_1)) / (2 * a);
  const double s_2 = s - (square(v_m) - square(v_2)) / (2 * d);

  if (s_2 >= s_1) {
    return {s_1, s_2};
  }

  const double y = (2 * d * s + square(v_2) - square(v_1)) / (2 * (a + d));
  return {y, y};
}

std::pair<double, double>
cda_rail::get_max_travel_time_acceleration_change_points(double v_1, double v_2,
                                                         double v_m, double a,
                                                         double d, double s) {
  // v_m is minimal speed in this case

  check_consistency_of_eom_input(v_1, v_2, a, d, s, s);
  if (v_m < 0) {
    throw exceptions::ConsistencyException(
        "v_m must be greater than or equal 0.");
  }

  const bool v_1_below_minimal_speed = v_1 < v_m;
  const bool v_2_below_minimal_speed = v_2 < v_m;

  const double a_1 = v_1_below_minimal_speed ? a : -d;
  const double a_2 = v_2_below_minimal_speed ? -d : a;

  const double s_1 = (square(v_m) - square(v_1)) / (2 * a_1);
  const double s_2 = s - (square(v_2) - square(v_m)) / (2 * a_2);

  if (s_2 >= s_1) {
    return {s_1, s_2};
  }

  if (v_1_below_minimal_speed && v_2_below_minimal_speed) {
    // Very short distance, train cannot reach minimal speed, hence same as
    // minimal time
    return get_min_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d,
                                                          s);
  }

  assert((!v_1_below_minimal_speed && !v_2_below_minimal_speed));

  const double y = (2 * a * s + square(v_1) - square(v_2)) /
                   (2 * (a + d)); // Distance at which accelerating starts if
  // minimal speed is not reached

  return {y, y};
}
