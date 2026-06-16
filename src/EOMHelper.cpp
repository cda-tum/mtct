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

template <typename... Args> /**
                             * @brief Rounds all arguments to zero below default
                             * tolerance.
                             *
                             * Modifies each argument in-place, treating values
                             * smaller than the default tolerance threshold as
                             * zero.
                             */
                            void round_with_default_eps(Args&... args) {
  // Refactoring helper: normalize multiple values with default tolerance.
  (round_small_numbers_to_zero_inplace(args), ...);
}

template <typename... Args> /**
                             * @brief Rounds all provided values to zero if they
                             * fall within a specified tolerance.
                             *
                             * @param eps Tolerance threshold for rounding.
                             * @param args Values to round in-place.
                             */
                            void round_with_eps(double eps, Args&... args) {
  // Refactoring helper: normalize multiple values with an explicit tolerance.
  (round_small_numbers_to_zero_inplace(args, eps), ...);
}

/**
 * @brief Computes the square of a number.
 *
 * @return double The square of the input value.
 */
[[nodiscard]] double square(double value) { return value * value; }

/**
 * @brief Computes braking distance for a given velocity and deceleration.
 *
 * Calculates the distance required to decelerate from velocity v to rest using
 * constant deceleration d. No input validation is performed; use the public
 * `braking_distance` function for validated computation.
 *
 * @param v Initial velocity.
 * @param d Constant deceleration rate.
 * @return The braking distance.
 */
[[nodiscard]] double braking_distance_unchecked(double v, double d) {
  return square(v) / (2 * d);
}

/**
 * @brief Computes a numerically stable form of a ratio.
 *
 * Returns `numerator / (scale * (sqrt(radicand) + denominator_offset))`.
 * This algebraic arrangement avoids cancellation errors in numerical
 * computation.
 *
 * @param numerator Dividend.
 * @param scale Scaling factor for the denominator.
 * @param radicand Value under the square root.
 * @param denominator_offset Added to the square root in the denominator.
 * @return The stable ratio.
 */
[[nodiscard]] double stable_ratio_with_sqrt(double numerator, double scale,
                                            double radicand,
                                            double denominator_offset) {
  // Shared stable form: numerator / (scale * (sqrt(radicand) + offset)).
  return numerator / (scale * (std::sqrt(radicand) + denominator_offset));
}

/**
 * @brief Computes the time to cover a distance under constant acceleration
 * using a numerically stable form.
 *
 * @return Time to traverse the distance; `0` if distance is zero.
 */
[[nodiscard]] double stable_phase_time(double distance, double initial_speed,
                                       double acceleration) {
  // Returns the time to cover `distance` from `initial_speed` at constant
  // `acceleration`. Original: t = (sqrt(v0^2 + 2*a*s) - v0) / a Stable: t =
  // 2*s / (sqrt(v0^2 + 2*a*s) + v0) Both are algebraically equivalent; the
  // stable form avoids cancellation.
  if (distance == 0) {
    return 0;
  }

  const double radicand =
      std::max(0.0, square(initial_speed) + (2 * acceleration * distance));
  return stable_ratio_with_sqrt(2 * distance, 1.0, radicand, initial_speed);
}

struct DistancePhases {
  double first;
  double second;
  double third;
};

/**
 * @brief Partitions traveled distance into three segments based on phase
 * boundaries.
 *
 * @param x Traveled distance.
 * @param s_1 Distance at end of first phase.
 * @param s_2 Distance at end of second phase.
 * @param s Total profile distance.
 * @return DistancePhases with `first`, `second`, `third` fields containing
 *         distance traveled in each phase.
 */
[[nodiscard]] DistancePhases
split_distance_by_change_points(double x, double s_1, double s_2, double s) {
  // Refactoring helper: split traveled distance into the 3 profile phases.
  return {
      .first  = std::min(x, s_1),
      .second = std::clamp(x - s_1, 0.0, s_2 - s_1),
      .third  = std::clamp(x - s_2, 0.0, s - s_2),
  };
}

/**
 * @brief Normalizes moving authority push inputs.
 *
 * Rounds `v_0`, `a`, and `s` to zero if below tolerance.
 * Rounds `d` to zero if slightly negative.
 */
void normalize_ma_push_inputs(double& v_0, double& a, double& d, double& s) {
  // Refactoring helper: centralize MA-input rounding and near-zero handling.
  round_with_eps(GRB_EPS, v_0, a, s);
  if (d < 0 && d > -GRB_EPS) {
    d = 0;
  }
}

/**
 * @brief Validates and normalizes an overlap braking distance.
 *
 * Rounds the input to zero if it falls within tolerance and validates
 * that the value is non-negative.
 *
 * @param obd Overlap braking distance.
 * @return The normalized overlap braking distance.
 */
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

/**
 * @brief Computes parameters for a three-phase motion profile.
 *
 * Determines phase accelerations, times, and distances for acceleration,
 * cruise, and deceleration phases. Validates feasibility within the given
 * distance.
 *
 * @return Profile structure containing all computed phase parameters.
 *
 * @throws ConsistencyException if the motion is infeasible within the available
 * distance.
 */
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

  return {.v_1        = v_1,
          .v_2        = v_2,
          .v_line     = v_line,
          .s          = s,
          .total_time = total_time,
          .a_1        = a_1,
          .a_2        = a_2,
          .t_1        = t_1,
          .t_2        = total_time - t_2_change,
          .s_1        = s_1};
}

/**
 * @brief Determines if a time query is beyond the speed profile endpoint with
 * numeric tolerance.
 *
 * @param total_time Nominal profile end time.
 * @param function_name Name of the calling function for error reporting.
 * @return true if time is treated as after the profile end within tolerance,
 * false if within tolerance of end.
 * @throws InvalidInputException if time exceeds the adjusted profile boundary
 * accounting for line speed precision.
 */
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
/**
 * @brief Computes the braking distance from a given speed under constant
 * deceleration.
 *
 * @param v Speed.
 * @param d Deceleration rate.
 * @return double The braking distance required to stop.
 */

double cda_rail::braking_distance(double v, double d) {
  exceptions::throw_if_negative(v, "Speed");
  exceptions::throw_if_non_positive(d, EPS, "Deceleration");

  return braking_distance_unchecked(v, d);
}

/**
 * @brief Computes the distance traveled when accelerating for a given time and
 * then braking to a complete stop.
 *
 * Accelerates from the initial speed for the specified duration, capped at the
 * maximum speed constraint, then adds the braking distance from the reached
 * speed.
 *
 * @return double The total distance traveled during acceleration and braking
 * phases.
 *
 * @throws exceptions::InvalidInputException If time is negative, maximum speed
 * is below initial speed, or acceleration or deceleration are non-positive.
 */
double cda_rail::max_braking_pos_after_dt_linear_movement(double v_0,
                                                          double v_max,
                                                          double a, double d,
                                                          double dt) {
  round_with_default_eps(v_0, v_max, a, d, dt);
  if (std::abs(v_0 - v_max) < EPS) {
    v_max = v_0;
  }

  exceptions::throw_if_less_than(dt, 0.0, "Time");
  exceptions::throw_if_negative(v_0, "Initial speed");
  if (v_max < v_0) {
    throw exceptions::InvalidInputException(
        "Maximum speed must be greater than or equal to initial speed.");
  }
  exceptions::throw_if_non_positive(a, "Acceleration");
  exceptions::throw_if_non_positive(d, "Deceleration");

  const double reached_speed      = std::min(v_max, v_0 + (a * dt));
  const double travelled_distance = (v_0 + reached_speed) * dt / 2;
  return travelled_distance + braking_distance(reached_speed, d);
}

// ---------------------------
// MINIMAL TRAVEL TIMES
/**
 * @brief Computes the minimum time to travel the full distance.
 *
 * @param v_1 Initial speed.
 * @param v_2 Final speed.
 * @param v_m Maximum line speed.
 * @param a Acceleration.
 * @param d Deceleration.
 * @param s Distance to travel.
 * @return double The minimum travel time.
 */

double cda_rail::min_travel_time(double v_1, double v_2, double v_m, double a,
                                 double d, double s) {
  return min_travel_time_from_start(v_1, v_2, v_m, a, d, s, s);
}

/**
 * @brief Computes the minimal time to travel a given distance on a segment.
 *
 * Calculates the minimum time required to travel distance @p x on a segment of
 * length @p s, starting at velocity @p v_1 and ending at velocity @p v_2,
 * under acceleration @p a and deceleration @p d constraints. The motion profile
 * consists of three phases: acceleration to maximum velocity @p v_m, cruise,
 * and deceleration to @p v_2.
 *
 * @param s Total segment length.
 * @param x Distance traveled from the start of the segment.
 * @return Minimal time to travel distance @p x.
 *
 * @throws ConsistencyException If inputs are inconsistent with the equations of
 * motion.
 */
double cda_rail::min_travel_time_from_start(double v_1, double v_2, double v_m,
                                            double a, double d, double s,
                                            double x) {
  check_consistency_of_eom_input(v_1, v_2, a, d, s, x);

  const auto [s_1, s_2] =
      get_min_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d, s);

  assert(s_2 >= s_1);

  const auto [x_1, x_2, x_3] = split_distance_by_change_points(x, s_1, s_2, s);

  const double v_t_squared = square(v_1) + (2 * a * s_1);
  const double v_t         = std::sqrt(v_t_squared);

  return stable_phase_time(x_1, v_1, a) + (x_2 == 0 ? 0 : x_2 / v_t) +
         stable_phase_time(x_3, v_t, -d);
}

/**
 * @brief Computes the minimal travel time from a position to the end of a
 * segment.
 *
 * Calculates the minimum time required to travel from position x to the end of
 * the segment (position s), starting with speed v_1 and arriving with speed
 * v_2, considering acceleration, deceleration, and a nominal speed v_m.
 *
 * @param v_1 Speed at position x.
 * @param v_2 Speed at position s.
 * @param v_m Nominal speed target.
 * @param a Maximum acceleration.
 * @param d Maximum deceleration.
 * @param s Total segment length.
 * @param x Current position.
 * @return Minimal travel time from position x to position s.
 */
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
/**
 * @brief Computes the maximum time to traverse the full segment distance.
 *
 * @param v_1 Initial speed.
 * @param v_2 Final speed.
 * @param v_m Minimal speed constraint.
 * @param a Acceleration.
 * @param d Deceleration.
 * @param s Segment distance.
 * @param stopping_allowed Whether the train may stop along the segment.
 * @return Maximum travel time over the full segment distance.
 */

double cda_rail::max_travel_time(double v_1, double v_2, double v_m, double a,
                                 double d, double s, bool stopping_allowed) {
  return max_travel_time_from_start(v_1, v_2, v_m, a, d, s, s,
                                    stopping_allowed);
}

/**
 * @brief Computes the maximum travel time from the segment's start to its end
 * without stopping.
 *
 * @return The maximum travel time.
 */
double cda_rail::max_travel_time_no_stopping(double v_1, double v_2, double v_m,
                                             double a, double d, double s) {
  return max_travel_time_from_start_no_stopping(v_1, v_2, v_m, a, d, s, s);
}

/**
 * @brief Calculates the maximum time to traverse a full distance when stopping
 * is allowed.
 *
 * @return Maximum travel time over the complete segment, or infinity if
 * stopping is feasible.
 */
double cda_rail::max_travel_time_stopping_allowed(double v_1, double v_2,
                                                  double a, double d,
                                                  double s) {
  return max_travel_time_from_start_stopping_allowed(v_1, v_2, a, d, s, s);
}

/**
 * @brief Computes the maximum time to reach distance x from the start.
 *
 * @return Maximum travel time from the start.
 */
double cda_rail::max_travel_time_from_start(double v_1, double v_2, double v_m,
                                            double a, double d, double s,
                                            double x, bool stopping_allowed) {
  return stopping_allowed
             ? max_travel_time_from_start_stopping_allowed(v_1, v_2, a, d, s, x)
             : max_travel_time_from_start_no_stopping(v_1, v_2, v_m, a, d, s,
                                                      x);
}

/**
 * @brief Computes the maximum time to travel from the segment start to a given
 * position without stopping.
 *
 * @param v_1 Initial velocity.
 * @param v_2 Final velocity.
 * @param v_m Minimal speed constraint.
 * @param a Acceleration magnitude.
 * @param d Deceleration magnitude.
 * @param s Segment length.
 * @param x Position within the segment.
 * @return Maximum feasible travel time to position @p x.
 */
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
  const double v_t_squared = square(v_1) + (2 * a_1 * s_1);
  const double v_t         = std::sqrt(v_t_squared);

  return stable_phase_time(x_1, v_1, a_1) + (x_2 == 0 ? 0 : x_2 / v_t) +
         stable_phase_time(x_3, v_t, a_3);
}

/**
 * @brief Computes the maximum time to reach a position from the front when
 * stopping is allowed.
 *
 * Returns infinity if the train can stop within the acceleration phase and the
 * query position is at or beyond that point, making the time constraint
 * unbounded. Otherwise delegates to the no-stopping calculation.
 *
 * @param v_1 Starting speed.
 * @param v_2 Ending speed.
 * @param a Acceleration.
 * @param d Deceleration.
 * @param s Edge length.
 * @param x Query position from the front.
 * @return Maximum time to reach position x, or infinity if stopping permits
 * unbounded transit time.
 */
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

/**
 * @brief Computes the maximum time to reach the segment end from a given
 * position, with optional stopping.
 *
 * @return Maximum travel time from position x to the segment end.
 */
double cda_rail::max_travel_time_to_end(double v_1, double v_2, double v_m,
                                        double a, double d, double s, double x,
                                        bool stopping_allowed) {
  return stopping_allowed
             ? max_travel_time_to_end_stopping_allowed(v_1, v_2, a, d, s, x)
             : max_travel_time_to_end_no_stopping(v_1, v_2, v_m, a, d, s, x);
}

/**
 * @brief Computes the maximum time to reach a position from the rear without
 * stopping.
 *
 * @return Maximum time to reach the position at distance @c x from the front.
 */
double cda_rail::max_travel_time_to_end_no_stopping(double v_1, double v_2,
                                                    double v_m, double a,
                                                    double d, double s,
                                                    double x) {
  // Same as maximal travel time going the reverse direction with acceleration
  // and deceleration swapped
  // NOLINTNEXTLINE(readability-suspicious-call-argument)
  return max_travel_time_from_start_no_stopping(v_2, v_1, v_m, d, a, s, s - x);
}

/**
 * @brief Determines the maximum travel time from a given position to the end of
 * a segment when stopping is allowed.
 *
 * @param v_1 Initial speed at position x.
 * @param v_2 Final speed at segment end.
 * @param a Acceleration magnitude.
 * @param d Deceleration magnitude.
 * @param s Segment length.
 * @param x Position from start of segment.
 * @return Maximum travel time from position x to segment end.
 */
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
/**
 * @brief Determines the minimal speed necessary for maximal-time travel.
 *
 * Computes the minimum speed the train must achieve to complete a journey
 * from v_1 to v_2 over distance s in maximum time, subject to minimum
 * speed constraint v_min.
 *
 * @return Minimum required intermediate speed.
 */

double cda_rail::minimal_line_speed(double v_1, double v_2, double v_min,
                                    double a, double d, double s) {
  const bool v_1_below_minimal_speed = v_1 < v_min;

  const auto [s_1, s_2] =
      get_max_travel_time_acceleration_change_points(v_1, v_2, v_min, a, d, s);

  (void)s_2; // unused

  const double a_1 = v_1_below_minimal_speed ? a : -d;
  return std::sqrt(square(v_1) + (2 * a_1 * s_1));
}

/**
 * @brief Computes the maximal line speed achievable for a minimal-time journey
 * profile.
 *
 * @param v_1 Initial speed.
 * @param v_2 Final speed.
 * @param v_max Target line speed.
 * @param a Acceleration.
 * @param d Deceleration.
 * @param s Segment distance.
 * @return The maximal line speed.
 */
double cda_rail::maximal_line_speed(double v_1, double v_2, double v_max,
                                    double a, double d, double s) {
  const auto [s_1, s_2] =
      get_min_travel_time_acceleration_change_points(v_1, v_2, v_max, a, d, s);

  assert(s_2 >= s_1);

  return std::sqrt(square(v_1) + (2 * a * s_1));
}

/**
 * @brief Determines the line speed required to achieve a target travel time.
 *
 * @param v_1 Initial speed.
 * @param v_2 Final speed.
 * @param v_min Minimum line speed.
 * @param v_max Maximum line speed.
 * @param a Acceleration.
 * @param d Deceleration.
 * @param s Edge length.
 * @param t Target travel time.
 * @return The line speed that achieves the target travel time, or 0 if the
 *         time exceeds the maximum feasible duration.
 */
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

/**
 * @brief Time for a train to traverse a distance with specified speed
 * constraints and kinematics.
 *
 * @return The total travel time.
 */
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

/**
 * @brief Computes position at a given time on an edge.
 *
 * Evaluates position along a constant-acceleration motion profile with initial
 * speed @p v_1, final speed @p v_2, and optional cruise at @p v_line.
 *
 * @param v_1 Initial speed.
 * @param v_2 Final speed.
 * @param v_line Cruise speed.
 * @param a Acceleration magnitude.
 * @param d Deceleration magnitude.
 * @param s Total segment distance.
 * @param t Query time.
 *
 * @return Position at time @p t. Returns total distance if @p t is at or beyond
 * the profile's end.
 *
 * @throw InvalidInputException if total travel time is negative.
 */
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

/**
 * @brief Computes the instantaneous velocity at a specified time along an edge
 * profile.
 *
 * Determines velocity during three-phase motion (acceleration to line speed,
 * cruising, deceleration) at time t. Returns the final velocity if t exceeds
 * the profile duration.
 *
 * @return The velocity at time t along the profile.
 *
 * @throws InvalidInputException if total travel time is negative.
 */
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
/**
 * @brief Computes the time to move a moving authority forward by a specified
 * distance.
 *
 * @return Time required to advance the moving authority by distance s.
 */

double cda_rail::min_time_to_push_ma_forward(double v_0, double a, double d,
                                             double s) {
  // How much time does a train need to move its moving authority forward by s
  // given initial speed v_0 and acceleration a and deceleration d?

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
      2 * d * s, 1.0, (2 * a_plus_d * a * d * s) + square(a_plus_d * v_0),
      a_plus_d * v_0);
}

/**
 * @brief Computes the minimum time to push a moving authority backward by a
 * given distance.
 *
 * @param v_0 Initial velocity.
 * @param a Acceleration.
 * @param d Deceleration.
 * @param s Distance to push the MA backward. Must not exceed v_0²/(2d).
 * @return Minimum time to push the MA backward by distance s.
 * @throws InvalidInputException if s exceeds v_0²/(2d).
 */
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
                                square(v_0) - (2 * a * d * s / a_plus_d), v_0);
}

/**
 * @brief Computes the minimum time to push the moving authority fully backward.
 *
 * @return Minimum time.
 *
 * @throws InvalidInputException if `v_0` is negative, `a` is negative, or `d`
 * is non-positive.
 */
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

/**
 * @brief Computes the minimum time from the train front to reach a moving
 * authority point.
 *
 * @return Minimum time to reach the MA point.
 */
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

/**
 * @brief Computes the maximum time from the train's front to a moving authority
 * point.
 *
 * @param s Edge length.
 * @param obd On-board distance offset to the moving authority point.
 * @param stopping_allowed If `true`, stopping is permitted; otherwise the train
 * must continue moving.
 * @return Maximum time to reach the moving authority point.
 */
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

/**
 * @brief Computes the maximum time to reach a moving authority point from the
 * front without stopping.
 *
 * Determines the maximal time required to travel from the front of the section
 * to a specified moving authority point given the kinematic constraints,
 * ensuring the train does not come to a complete stop.
 *
 * @param v_1 Initial speed at the front.
 * @param v_2 Target speed at the rear.
 * @param v_m Minimal speed constraint for the maximal-time scenario.
 * @param a Acceleration rate.
 * @param d Deceleration rate.
 * @param s Section distance.
 * @param obd On-board device distance defining the moving authority point
 * location.
 *
 * @return Maximum time to reach the moving authority point without stopping.
 */
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

  if (const double ubd_s2 = ma_point - (s_2 + bd_t); ubd_s2 > GRB_EPS) {
    return max_travel_time_from_start_no_stopping(v_1, v_2, v_m, a, d, s, s_2) +
           min_time_to_push_ma_forward(v_t, a, d, ubd_s2);
  }
  const double ubd_s1 = ma_point - (s_1 + bd_t);
  if (ubd_s1 > GRB_EPS) {
    return max_travel_time_from_start_no_stopping(v_1, v_2, v_m, a, d, s,
                                                  s_1 + ubd_s1);
  }

  if (!v1_below_minimal_speed && std::abs(ubd_s1) <= GRB_EPS) {
    return 0;
  }
  assert(v1_below_minimal_speed);

  return min_time_to_push_ma_forward(v_1, a, d, ma_point - bd_1);
}

/**
 * @brief Computes the maximum time from the train front to a moving authority
 * point when stopping is allowed.
 *
 * @return The maximum time to reach the MA point, or infinity if stopping at
 * the MA point is feasible.
 */
double cda_rail::max_time_from_front_to_ma_point_stopping_allowed(
    double v_1, double v_2, double a, double d, double s, double obd) {
  if (max_travel_time_stopping_allowed(v_1, v_2, a, d, s) >=
      std::numeric_limits<double>::infinity()) {
    return std::numeric_limits<double>::infinity();
  }
  return max_time_from_front_to_ma_point_no_stopping(v_1, v_2, 0, a, d, s, obd);
}

/**
 * @brief Computes the minimum time from the rear of a train to reach a moving
 * authority point.
 *
 * @param obd On-board distance from the train front to the moving authority
 * point.
 * @param strategy Timing strategy; must be `ExtremeProfiles`.
 * @return Time in seconds.
 * @throws InvalidInputException if `strategy` is not `ExtremeProfiles`.
 */
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

/**
 * @brief Computes the maximum time from the train's rear to reach a moving
 * authority point.
 *
 * Enforces that `strategy` must be `MATimingStrategy::ExtremeProfiles`;
 * otherwise throws `InvalidInputException`.
 *
 * @return Maximum time from the extreme profile-based computations.
 */
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

/**
 * @brief Calculates the minimal time for the rear of the train to advance from
 * the moving authority point to the segment end.
 *
 * @param obd On-board distance (train length).
 * @return Minimal time for the rear to reach the segment end from the MA point.
 */
double cda_rail::min_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                                        double v_m, double a,
                                                        double d, double s,
                                                        double obd) {
  return min_travel_time(v_1, v_2, v_m, a, d, s) -
         min_time_from_front_to_ma_point(v_1, v_2, v_m, a, d, s, obd);
}

/**
 * @brief Computes the maximum time for the rear to reach a Moving Authority
 * point.
 *
 * @param v_m Minimal speed constraint.
 * @param obd Offset backward distance to the Moving Authority point.
 * @return double Maximum time for the rear to reach the MA point.
 */
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
/**
 * @brief Validates and normalizes kinematic input parameters for motion
 * equations.
 *
 * Rounds small values toward zero using GRB_EPS tolerance. Validates that all
 * parameters meet kinematic constraints: non-negativity, strictly positive
 * acceleration/deceleration, position ≤ distance, and feasibility per equations
 * of motion. All parameters are modified in-place (rounded).
 *
 * @throws ConsistencyException if any constraint is violated.
 */

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

/**
 * @brief Determines if a velocity transition is feasible by equations of
 * motion.
 *
 * @return `true` if the train can transition from @p v_1 to @p v_2 over
 * distance @p s using the available acceleration @p a or deceleration @p d,
 *         `false` otherwise.
 */
bool cda_rail::possible_by_eom(double v_1, double v_2, double a, double d,
                               double s) {
  return v_1 <= v_2 ? square(v_2) - square(v_1) <= (2 * a * s) + GRB_EPS
                    : square(v_1) - square(v_2) <= (2 * d * s) + GRB_EPS;
}

/**
 * @brief Computes distances marking acceleration changes in a minimal-time
 * profile.
 *
 * Determines where the train transitions between acceleration, constant speed,
 * and deceleration phases when traveling from v_1 to v_2 at line speed v_m
 * over the given distance.
 *
 * @param v_m Target cruise speed; must be positive and at least v_1 and v_2.
 *
 * @return A pair {s_1, s_2} of distances where acceleration changes.
 *         Returns two equal values if the cruise phase is infeasible.
 *
 * @throws ConsistencyException if v_m is non-positive, if v_m is less than v_1
 * or v_2, or if the input profile is inconsistent.
 */
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
  const double s_2 = s - ((square(v_m) - square(v_2)) / (2 * d));

  if (s_2 >= s_1) {
    return {s_1, s_2};
  }

  const double y = ((2 * d * s) + square(v_2) - square(v_1)) / (2 * (a + d));
  return {y, y};
}

/**
 * @brief Computes acceleration change-point distances for maximal travel time
 * with a minimal-speed constraint.
 *
 * Determines the distance points where the acceleration profile changes when
 * traveling from v_1 to v_2 over distance s, maximizing travel time while
 * respecting a minimum speed v_m.
 *
 * @param v_1 Initial velocity.
 * @param v_2 Final velocity.
 * @param v_m Minimum speed; must be non-negative.
 * @param a Acceleration magnitude.
 * @param d Deceleration magnitude.
 * @param s Total distance.
 * @return Pair of change-point distances {s_1, s_2}. If the profile reaches the
 * minimum speed, s_1 marks the transition to minimum speed and s_2 marks the
 * start of deceleration to v_2. If minimal speed is unattainable, both values
 * are equal, representing a single transition point.
 *
 * @throws ConsistencyException if inputs violate consistency or feasibility
 * constraints.
 */
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

  if (const double s_2 = s - ((square(v_2) - square(v_m)) / (2 * a_2));
      s_2 >= s_1) {
    return {s_1, s_2};
  }

  if (v_1_below_minimal_speed && v_2_below_minimal_speed) {
    // Very short distance, train cannot reach minimal speed, hence same as
    // minimal time
    return get_min_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d,
                                                          s);
  }

  assert((!v_1_below_minimal_speed && !v_2_below_minimal_speed));

  const double y = ((2 * a * s) + square(v_1) - square(v_2)) /
                   (2 * (a + d)); // Distance at which accelerating starts if
  // minimal speed is not reached

  return {y, y};
}
