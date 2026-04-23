#pragma once

#include <cstdint>
#include <utility>

// EOM = Equations of Motion

namespace cda_rail {

/**
 * @brief Helpers for 1D train motion under constant-acceleration bounds.
 *
 * All functions in this header use a simplified longitudinal motion model:
 * acceleration is piecewise constant and limited to `+a` (accelerating),
 * `0` (coasting), or `-d` (braking). This yields quadratic kinematic
 * relations in position, speed, and time.
 *
 * Typical use case: edge length and boundary speeds are known (`v_1`, `v_2`,
 * `s`), while the intermediate speed profile is not. This API provides
 * physically consistent profile choices, especially:
 * - minimum-time envelopes,
 * - maximum-time envelopes (with or without full stop),
 * - line-speed and moving-authority timing relations.
 *
 * Terminology used below:
 * - braking distance at speed `v`: `v^2 / (2*d)`.
 * - braking point (or moving-authority point): position where the train would
 *   come to rest if full braking started immediately.
 */

// ---------------------------
// GENERAL MEASURES
// ---------------------------

/**
 * @brief Computes braking distance for constant deceleration.
 *
 * Uses the kinematic relation
 * `bd = v^2 / (2 * d)`
 * where @p v is the current speed and @p d is the available deceleration.
 *
 * @param v Velocity in m/s.
 * @param d Deceleration in m/s^2.
 * @return Braking distance in metres.
 * @pre `v >= 0` and `d > 0`.
 * @throws cda_rail::exceptions::InvalidInputException If @p v is negative
 *         or @p d is not strictly positive.
 */
double braking_distance(double v, double d);

/**
 * @brief Computes the maximal forward shift of the braking point after @p dt.
 *
 * The train is assumed to accelerate linearly for @p dt seconds, capped at
 * @p v_max, and then brake immediately with deceleration @p d.
 *
 * Returned value =
 * - distance traveled during the acceleration phase over @p dt, plus
 * - braking distance at the reached speed.
 *
 * @param v_0 Initial speed in m/s.
 * @param v_max Speed cap in m/s.
 * @param a Acceleration in m/s^2.
 * @param d Deceleration in m/s^2.
 * @param dt Time horizon in seconds.
 * @return Maximum reachable braking-point position relative to the current
 *         front position (in metres) after @p dt.
 * @pre `dt >= 0`, `v_0 >= 0`, `v_max >= v_0`, `a > 0`, `d > 0`.
 * @throws cda_rail::exceptions::InvalidInputException If any precondition is
 *         violated.
 */
double max_braking_pos_after_dt_linear_movement(double v_0, double v_max,
                                                double a, double d, int dt);

// ---------------------------
// MINIMAL TRAVEL TIMES
//
// (assuming acceleration - travel at max speed - deceleration)
// ---------------------------

/**
 * @brief Returns the minimal time to traverse an edge of length @p s.
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Upper speed bound (line speed) in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @return Minimal feasible traversal time in seconds.
 * @pre Same consistency requirements as `min_travel_time_from_start(...)`
 *      with `x = s`.
 * @throws cda_rail::exceptions::ConsistencyException If the speed profile or
 *         input range is not feasible under equations of motion.
 */
double min_travel_time(double v_1, double v_2, double v_m, double a, double d,
                       double s);

/**
 * @brief Returns minimal time to reach position @p x from the edge start.
 *
 * The profile is the time-optimal one under acceleration bound @p a,
 * deceleration bound @p d, and speed cap @p v_m.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Upper speed bound (line speed) in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param x Distance from start in metres (`0` at start, `s` at end).
 * @return Minimal time in seconds to reach @p x.
 * @pre `v_1 >= 0`, `v_2 >= 0`, `a > 0`, `d > 0`, `s >= 0`, `0 <= x <= s`, and
 *      transition from @p v_1 to @p v_2 over @p s is physically feasible.
 * @pre `v_m > 0` and `v_m >= max(v_1, v_2)`.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double min_travel_time_from_start(double v_1, double v_2, double v_m, double a,
                                  double d, double s, double x);

/**
 * @brief Returns minimal remaining time from position @p x to edge end.
 *
 * Equivalent to reversing direction and swapping acceleration/deceleration.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Upper speed bound (line speed) in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param x Distance from start in metres.
 * @return Minimal time in seconds from @p x to the edge end.
 * @pre Same feasibility requirements as `min_travel_time_from_start(...)`.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double min_travel_time_to_end(double v_1, double v_2, double v_m, double a,
                              double d, double s, double x);

// ----------------------------
// MAX TRAVEL TIMES
//
// (assuming deceleration - travel at min speed - acceleration)
// ----------------------------

/**
 * @brief Returns maximal traversal time for an edge.
 *
 * Dispatches to the stopping or non-stopping variant based on
 * @p stopping_allowed.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Lower speed bound in m/s if stopping is not allowed.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param stopping_allowed If `true`, full stop may occur on the edge.
 * @return Maximal feasible traversal time in seconds; may be `infinity`.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are infeasible.
 */
double max_travel_time(double v_1, double v_2, double v_m, double a, double d,
                       double s, bool stopping_allowed);

/**
 * @brief Returns maximal traversal time without allowing a full stop.
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Lower speed bound in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @return Maximal finite traversal time in seconds.
 * @pre Same consistency requirements as
 * `max_travel_time_from_start_no_stopping` with `x = s`.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double max_travel_time_no_stopping(double v_1, double v_2, double v_m, double a,
                                   double d, double s);

/**
 * @brief Returns maximal traversal time when stopping is allowed.
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @return Maximal traversal time in seconds; returns `infinity` if the train
 *         can come to a full stop before the constrained continuation point.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double max_travel_time_stopping_allowed(double v_1, double v_2, double a,
                                        double d, double s);

/**
 * @brief Returns maximal time to reach position @p x from edge start.
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Lower speed bound in m/s if stopping is not allowed.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param x Distance from start in metres.
 * @param stopping_allowed If `true`, full stop may occur.
 * @return Maximal time in seconds to reach @p x; may be `infinity`.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double max_travel_time_from_start(double v_1, double v_2, double v_m, double a,
                                  double d, double s, double x,
                                  bool stopping_allowed);

/**
 * @brief Returns maximal time to reach @p x without allowing a full stop.
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Lower speed bound in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param x Distance from start in metres.
 * @return Maximal finite time in seconds to reach @p x.
 * @pre `v_1 >= 0`, `v_2 >= 0`, `a > 0`, `d > 0`, `s >= 0`, `0 <= x <= s`, and
 *      transition from @p v_1 to @p v_2 over @p s is physically feasible.
 * @pre `v_m >= 0`.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double max_travel_time_from_start_no_stopping(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double x);

/**
 * @brief Returns maximal time to reach @p x when stopping is allowed.
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param x Distance from start in metres.
 * @return Maximal time in seconds; may be `infinity` once a full stop is
 *         reachable before (or at) @p x.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double max_travel_time_from_start_stopping_allowed(double v_1, double v_2,
                                                   double a, double d, double s,
                                                   double x);

/**
 * @brief Returns maximal remaining time from @p x to edge end.
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Lower speed bound in m/s if stopping is not allowed.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param x Distance from start in metres.
 * @param stopping_allowed If `true`, full stop may occur.
 * @return Maximal remaining time in seconds; may be `infinity`.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double max_travel_time_to_end(double v_1, double v_2, double v_m, double a,
                              double d, double s, double x,
                              bool stopping_allowed);

/**
 * @brief Returns maximal remaining time from @p x to end without stopping.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double max_travel_time_to_end_no_stopping(double v_1, double v_2, double v_m,
                                          double a, double d, double s,
                                          double x);

/**
 * @brief Returns maximal remaining time from @p x to end when stopping is
 *        allowed.
 * @return Maximal remaining time in seconds; may be `infinity`.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are
 *         inconsistent or infeasible.
 */
double max_travel_time_to_end_stopping_allowed(double v_1, double v_2, double a,
                                               double d, double s, double x);

// ---------------------------
// LINE SPEED CALCULATIONS
//
// (assuming speed change to line speed - travel at line speed - speed change to
// final speed)
// ---------------------------

/**
 * @brief Returns the minimum line speed reached on the maximal-time profile.
 *
 * If the edge is too short to reach @p v_min, the returned value is the
 * actual turning speed reached by the profile.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_min Desired lower line speed bound in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @return Reached minimum line speed in m/s.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are invalid.
 */
double minimal_line_speed(double v_1, double v_2, double v_min, double a,
                          double d, double s);

/**
 * @brief Returns the maximum line speed reached on the minimal-time profile.
 *
 * If the edge is too short to reach @p v_max, the returned value is the
 * actual turning speed reached by the profile.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_max Desired upper line speed bound in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @return Reached maximum line speed in m/s.
 * @throws cda_rail::exceptions::ConsistencyException If inputs are invalid.
 */
double maximal_line_speed(double v_1, double v_2, double v_max, double a,
                          double d, double s);

/**
 * @brief Finds a line speed that yields traversal time @p t.
 *
 * The function searches line speed in `[minimal_line_speed,
 * maximal_line_speed]` via bisection. If @p t exceeds the maximal feasible
 * no-stop travel time, returns `0` as a sentinel for "no feasible line speed".
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_min Lower bound for candidate line speed in m/s.
 * @param v_max Upper bound for candidate line speed in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param t Desired traversal time in seconds.
 * @return Feasible line speed in m/s, or `0` if no finite no-stop solution
 *         exists for @p t.
 * @throws cda_rail::exceptions::ConsistencyException If intermediate EOM checks
 *         fail for the provided parameters.
 */
double get_line_speed(double v_1, double v_2, double v_min, double v_max,
                      double a, double d, double s, double t);

/**
 * @brief Computes traversal time for a fixed line speed profile.
 *
 * Profile structure:
 * - speed change from @p v_1 to @p v_line,
 * - optional cruising at @p v_line,
 * - speed change from @p v_line to @p v_2.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_line Intermediate line speed in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @return Total traversal time in seconds.
 * @pre `v_1 >= 0`, `v_2 >= 0`, `v_line > 0`, `a > 0`, `d > 0`, `s >= 0`.
 * @throws cda_rail::exceptions::InvalidInputException If input domain checks
 *         fail.
 * @throws cda_rail::exceptions::ConsistencyException If the profile cannot be
 *         realized over distance @p s.
 */
double time_on_edge(double v_1, double v_2, double v_line, double a, double d,
                    double s);

/**
 * @brief Returns position on the edge at time @p t for a fixed line speed
 *        profile.
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_line Intermediate line speed in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param t Time in seconds from edge entry.
 * @return Position in metres from edge start.
 * @pre Inputs must define a feasible `time_on_edge(...)` profile.
 * @pre `t` must not exceed total travel time (up to numerical tolerance).
 * @throws cda_rail::exceptions::InvalidInputException If @p t is inconsistent
 *         with the total profile duration or input domain checks fail.
 * @throws cda_rail::exceptions::ConsistencyException If profile realization
 *         fails in `time_on_edge(...)`.
 */
double pos_on_edge_at_time(double v_1, double v_2, double v_line, double a,
                           double d, double s, double t);

/**
 * @brief Returns velocity on the edge at time @p t for a fixed line speed
 *        profile.
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_line Intermediate line speed in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param t Time in seconds from edge entry.
 * @return Instantaneous speed in m/s.
 * @pre Inputs must define a feasible `time_on_edge(...)` profile.
 * @pre `t` must not exceed total travel time (up to numerical tolerance).
 * @throws cda_rail::exceptions::InvalidInputException If @p t is inconsistent
 *         with the total profile duration or input domain checks fail.
 * @throws cda_rail::exceptions::ConsistencyException If profile realization
 *         fails in `time_on_edge(...)`.
 */
double vel_on_edge_at_time(double v_1, double v_2, double v_line, double a,
                           double d, double s, double t);

// ---------------------------
// MOVING AUTHORITY CALCULATIONS
// ---------------------------

// moving MA point

/**
 * @brief Minimal time to move the moving-authority point forward by @p s.
 *
 * "Forward" refers to increasing MA distance from the train's initial MA,
 * accounting for both train displacement and the changed braking distance.
 *
 * @param v_0 Initial speed in m/s.
 * @param a Acceleration in m/s^2 (may be zero).
 * @param d Deceleration in m/s^2.
 * @param s Required MA forward shift in metres.
 * @return Minimal time in seconds.
 * @pre `v_0 >= 0`, `a >= 0`, `d > 0`, `s >= 0`.
 * @throws cda_rail::exceptions::InvalidInputException If inputs violate the
 *         preconditions.
 */
double min_time_to_push_ma_forward(double v_0, double a, double d, double s);

/**
 * @brief Minimal time to move the moving-authority point backward by @p s.
 *
 * "Backward" means reducing MA overlap by @p s relative to the current state.
 *
 * @param v_0 Initial speed in m/s.
 * @param a Acceleration in m/s^2 (may be zero).
 * @param d Deceleration in m/s^2.
 * @param s Desired MA backward shift in metres.
 * @return Minimal time in seconds.
 * @pre `v_0 >= 0`, `a >= 0`, `d > 0`, `s >= 0`, and
 *      `s <= v_0^2 / (2 * d)`.
 * @throws cda_rail::exceptions::InvalidInputException If inputs violate the
 *         preconditions.
 */
double min_time_to_push_ma_backward(double v_0, double a, double d, double s);

/**
 * @brief Minimal time to move MA fully backward to the initial train front.
 *
 * Equivalent to `min_time_to_push_ma_backward(v_0, a, d, v_0^2/(2*d))`.
 *
 * @param v_0 Initial speed in m/s.
 * @param a Acceleration in m/s^2 (may be zero).
 * @param d Deceleration in m/s^2.
 * @return Minimal time in seconds.
 * @pre `v_0 >= 0`, `a >= 0`, `d > 0`.
 * @throws cda_rail::exceptions::InvalidInputException If inputs violate the
 *         preconditions.
 */
double min_time_to_push_ma_fully_backward(double v_0, double a, double d);

// reaching MA

/**
 * @brief Minimal time from train front to a target MA point.
 *
 * This function uses the same MA convention as the tests and implementation:
 * the target MA point is encoded by overlap braking distance @p obd.
 * Intuitively, @p obd measures how much of the final braking-distance
 * reference remains after accounting for the MA point location.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Upper speed bound for the minimum-time profile in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @param obd Overlap braking distance in metres (`obd >= 0`).
 * @return Minimal time in seconds until train front reaches the MA point.
 * @throws cda_rail::exceptions::InvalidInputException If @p obd is negative.
 * @throws cda_rail::exceptions::ConsistencyException If the requested MA point
 *         is inconsistent with the speed profile or edge geometry.
 */
double min_time_from_front_to_ma_point(double v_1, double v_2, double v_m,
                                       double a, double d, double s,
                                       double obd);

/**
 * @brief Maximal time from train front to a target MA point.
 *
 * Dispatches to stopping or non-stopping variant based on
 * @p stopping_allowed.
 *
 * Uses the same MA-point encoding via @p obd as
 * `min_time_from_front_to_ma_point(...)`.
 *
 * @return Maximal time in seconds; may be `infinity` when stopping is allowed.
 * @throws cda_rail::exceptions::InvalidInputException If @p obd is negative.
 * @throws cda_rail::exceptions::ConsistencyException If EOM consistency fails.
 */
double max_time_from_front_to_ma_point(double v_1, double v_2, double v_m,
                                       double a, double d, double s, double obd,
                                       bool stopping_allowed);

/**
 * @brief Maximal time from train front to MA point without full stop.
 * @return Maximal finite time in seconds.
 * @throws cda_rail::exceptions::InvalidInputException If @p obd is negative.
 * @throws cda_rail::exceptions::ConsistencyException If EOM consistency fails.
 */
double max_time_from_front_to_ma_point_no_stopping(double v_1, double v_2,
                                                   double v_m, double a,
                                                   double d, double s,
                                                   double obd);

/**
 * @brief Maximal time from train front to MA point when stopping is allowed.
 * @return Maximal time in seconds; may be `infinity`.
 * @throws cda_rail::exceptions::InvalidInputException If @p obd is negative.
 * @throws cda_rail::exceptions::ConsistencyException If EOM consistency fails.
 */
double max_time_from_front_to_ma_point_stopping_allowed(double v_1, double v_2,
                                                        double a, double d,
                                                        double s, double obd);

/**
 * @brief Strategy selector for MA timing from train rear.
 */
enum class MATimingStrategy : std::uint8_t { ExtremeProfiles = 0 };

/**
 * @brief Minimal time from train rear to MA point under a strategy.
 *
 * For `ExtremeProfiles`, both extreme speed envelopes (using @p v_min and
 * @p v_max) are evaluated and the smaller resulting time is returned.
 *
 * @param strategy Strategy to combine profile times.
 * @return Minimal rear-to-MA time in seconds.
 * @throws cda_rail::exceptions::InvalidInputException If @p strategy is not
 *         supported.
 * @throws cda_rail::exceptions::InvalidInputException If delegated MA helper
 *         input checks fail.
 * @throws cda_rail::exceptions::ConsistencyException If delegated EOM checks
 *         fail.
 */
double min_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd,
    MATimingStrategy strategy = MATimingStrategy::ExtremeProfiles);

/**
 * @brief Maximal time from train rear to MA point under a strategy.
 *
 * For `ExtremeProfiles`, both extreme speed envelopes (using @p v_min and
 * @p v_max) are evaluated and the larger resulting time is returned.
 *
 * @param strategy Strategy to combine profile times.
 * @return Maximal rear-to-MA time in seconds.
 * @throws cda_rail::exceptions::InvalidInputException If @p strategy is not
 *         supported.
 * @throws cda_rail::exceptions::InvalidInputException If delegated MA helper
 *         input checks fail.
 * @throws cda_rail::exceptions::ConsistencyException If delegated EOM checks
 *         fail.
 */
double max_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd,
    MATimingStrategy strategy = MATimingStrategy::ExtremeProfiles);

/**
 * @brief Rear-to-MA time under a single minimum-time speed profile.
 *
 * Computed as:
 * `min_travel_time(...) - min_time_from_front_to_ma_point(...)`.
 */
double min_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double obd);

/**
 * @brief Rear-to-MA time under a single maximum-time no-stopping profile.
 *
 * Computed as:
 * `max_travel_time_no_stopping(...) -`
 * `max_time_from_front_to_ma_point_no_stopping(...)`.
 */
double max_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double obd);

// ---------------------------
// HELPER
// ---------------------------

/**
 * @brief Normalizes and validates common EOM inputs in place.
 *
 * Small values near zero are rounded to zero (using project tolerance), then
 * consistency checks are applied.
 *
 * @param v_1 Entry speed in m/s (updated in place).
 * @param v_2 Exit speed in m/s (updated in place).
 * @param a Acceleration in m/s^2 (updated in place).
 * @param d Deceleration in m/s^2 (updated in place).
 * @param s Total edge length in metres (updated in place).
 * @param x Position on edge in metres (updated in place).
 * @pre Inputs should be finite real values.
 * @throws cda_rail::exceptions::ConsistencyException If any value is negative
 *         after rounding, if `a <= 0` or `d <= 0`, if `x > s`, or if the entry
 *         and exit speeds are not feasible over @p s.
 */
void check_consistency_of_eom_input(double& v_1, double& v_2, double& a,
                                    double& d, double& s, double& x);

/**
 * @brief Checks whether speed transition is feasible by equations of motion.
 *
 * For acceleration (`v_2 >= v_1`), checks against acceleration bound @p a.
 * For deceleration (`v_2 < v_1`), checks against deceleration bound @p d.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Distance in metres.
 * @return `true` if transition is feasible within tolerance.
 */
bool possible_by_eom(double v_1, double v_2, double a, double d, double s);

/**
 * @brief Returns profile change-point distances for minimum-time motion.
 *
 * Returned pair `{s_1, s_2}` marks transition points between
 * accelerate / cruise / decelerate phases. If no cruise phase exists,
 * `s_1 == s_2`.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Upper speed bound in m/s.
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @return Pair of change-point distances in metres.
 * @pre `v_m > 0` and `v_m >= max(v_1, v_2)`.
 * @throws cda_rail::exceptions::ConsistencyException If EOM consistency fails
 *         or @p v_m violates preconditions.
 */
[[nodiscard]] std::pair<double, double>
get_min_travel_time_acceleration_change_points(double v_1, double v_2,
                                               double v_m, double a, double d,
                                               double s);

/**
 * @brief Returns profile change-point distances for maximum-time motion.
 *
 * Returned pair `{s_1, s_2}` marks transition points between
 * decelerate / cruise / accelerate phases (or the corresponding mirrored
 * behavior when speeds are below @p v_m). If no cruise phase exists,
 * `s_1 == s_2`.
 *
 * @param v_1 Entry speed in m/s.
 * @param v_2 Exit speed in m/s.
 * @param v_m Lower speed bound in m/s (`v_m >= 0`).
 * @param a Maximum acceleration in m/s^2.
 * @param d Maximum deceleration in m/s^2.
 * @param s Edge length in metres.
 * @return Pair of change-point distances in metres.
 * @throws cda_rail::exceptions::ConsistencyException If EOM consistency fails
 *         or @p v_m is negative.
 */
[[nodiscard]] std::pair<double, double>
get_max_travel_time_acceleration_change_points(double v_1, double v_2,
                                               double v_m, double a, double d,
                                               double s);

} // namespace cda_rail
