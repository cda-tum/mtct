#pragma once

#include <cstdint>
#include <utility>

// EOM = Equations of Motion

namespace cda_rail {

// ---------------------------
// GENERAL MEASURES
// ---------------------------

/**
 * @brief Braking distance for a given speed and (maximal) deceleration.
 *
 * Assuming constant deceleration, the braking distance (bd) is given by:
 * bd = v^2 / 2d
 *
 * @param v Velocity (in m/s)
 * @param d (Maximal) deceleration (in m/s^2)
 * @return Braking distance
 */
double braking_distance(double v, double d);

/**
 * @brief Maximal displacement of braking point (moving authority)
 *
 * Calculates the maximum position of the braking point after dt seconds.
 * The movement within the dt seconds is linear, i.e., the acceleration is
 * constant.
 *
 * @param v_0 Initial speed (in m/s)
 * @param v_max Maximum speed (in m/s)
 * @param a Acceleration (in m/s^2)
 * @param d Deceleration (in m/s^2)
 * @param dt Time (in s)
 * @return Point at which the train can come to a full stop after accelerating
 * for dt seconds.
 */
double max_braking_pos_after_dt_linear_movement(double v_0, double v_max,
                                                double a, double d, int dt);

// ---------------------------
// MINIMAL TRAVEL TIMES
//
// (assuming acceleration - travel at max speed - deceleration)
// ---------------------------

double min_travel_time(double v_1, double v_2, double v_m, double a, double d,
                       double s);
double min_travel_time_from_start(double v_1, double v_2, double v_m, double a,
                                  double d, double s, double x);
double min_travel_time_to_end(double v_1, double v_2, double v_m, double a,
                              double d, double s, double x);

// ----------------------------
// MAX TRAVEL TIMES
//
// (assuming deceleration - travel at min speed - acceleration)
// ----------------------------
double max_travel_time(double v_1, double v_2, double v_m, double a, double d,
                       double s, bool stopping_allowed);
double max_travel_time_no_stopping(double v_1, double v_2, double v_m, double a,
                                   double d, double s);
double max_travel_time_stopping_allowed(double v_1, double v_2, double a,
                                        double d, double s);

double max_travel_time_from_start(double v_1, double v_2, double v_m, double a,
                                  double d, double s, double x,
                                  bool stopping_allowed);
double max_travel_time_from_start_no_stopping(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double x);
double max_travel_time_from_start_stopping_allowed(double v_1, double v_2,
                                                   double a, double d, double s,
                                                   double x);

double max_travel_time_to_end(double v_1, double v_2, double v_m, double a,
                              double d, double s, double x,
                              bool stopping_allowed);
double max_travel_time_to_end_no_stopping(double v_1, double v_2, double v_m,
                                          double a, double d, double s,
                                          double x);
double max_travel_time_to_end_stopping_allowed(double v_1, double v_2, double a,
                                               double d, double s, double x);

// ---------------------------
// LINE SPEED CALCULATIONS
//
// (assuming speed change to line speed - travel at line speed - speed change to
// final speed)
// ---------------------------
double minimal_line_speed(double v_1, double v_2, double v_min, double a,
                          double d, double s);
double maximal_line_speed(double v_1, double v_2, double v_max, double a,
                          double d, double s);

double get_line_speed(double v_1, double v_2, double v_min, double v_max,
                      double a, double d, double s, double t);

double time_on_edge(double v_1, double v_2, double v_line, double a, double d,
                    double s);

double pos_on_edge_at_time(double v_1, double v_2, double v_line, double a,
                           double d, double s, double t);
double vel_on_edge_at_time(double v_1, double v_2, double v_line, double a,
                           double d, double s, double t);

// ---------------------------
// MOVING AUTHORITY CALCULATIONS
// ---------------------------

// moving MA point
double min_time_to_push_ma_forward(double v_0, double a, double d, double s);
double min_time_to_push_ma_backward(double v_0, double a, double d, double s);
double min_time_to_push_ma_fully_backward(double v_0, double a, double d);

// reaching MA
double min_time_from_front_to_ma_point(double v_1, double v_2, double v_m,
                                       double a, double d, double s,
                                       double obd);
double max_time_from_front_to_ma_point(double v_1, double v_2, double v_m,
                                       double a, double d, double s, double obd,
                                       bool stopping_allowed);
double max_time_from_front_to_ma_point_no_stopping(double v_1, double v_2,
                                                   double v_m, double a,
                                                   double d, double s,
                                                   double obd);
double max_time_from_front_to_ma_point_stopping_allowed(double v_1, double v_2,
                                                        double a, double d,
                                                        double s, double obd);

enum class MATimingStrategy : std::uint8_t { ExtremeProfiles = 0 };
double min_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd,
    MATimingStrategy strategy = MATimingStrategy::ExtremeProfiles);
double max_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd,
    MATimingStrategy strategy = MATimingStrategy::ExtremeProfiles);

double min_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double obd);

double max_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double obd);

// ---------------------------
// HELPER
// ---------------------------

void check_consistency_of_eom_input(double& v_1, double& v_2, double& a,
                                    double& d, double& s, double& x);
bool possible_by_eom(double v_1, double v_2, double a, double d, double s);

[[nodiscard]] std::pair<double, double>
get_min_travel_time_acceleration_change_points(double v_1, double v_2,
                                               double v_m, double a, double d,
                                               double s);
[[nodiscard]] std::pair<double, double>
get_max_travel_time_acceleration_change_points(double v_1, double v_2,
                                               double v_m, double a, double d,
                                               double s);

} // namespace cda_rail
