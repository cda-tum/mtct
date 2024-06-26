#pragma once

#include <utility>

// EOM = Equations of Motion

namespace cda_rail {
bool possible_by_eom(double v_1, double v_2, double a, double d, double s);
void check_consistency_of_eom_input(double& v_1, double& v_2, double& a,
                                    double& d, double& s, double& x);

[[nodiscard]] std::pair<double, double>
get_min_travel_time_acceleration_change_points(double v_1, double v_2,
                                               double v_m, double a, double d,
                                               double s);
[[nodiscard]] std::pair<double, double>
get_max_travel_time_acceleration_change_points(double v_1, double v_2,
                                               double v_m, double a, double d,
                                               double s);

double min_travel_time_from_start(double v_1, double v_2, double v_m, double a,
                                  double d, double s, double x);
double min_travel_time(double v_1, double v_2, double v_m, double a, double d,
                       double s);
double min_travel_time_to_end(double v_1, double v_2, double v_m, double a,
                              double d, double s, double x);

double max_travel_time_from_start_no_stopping(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double x);
double max_travel_time_no_stopping(double v_1, double v_2, double v_m, double a,
                                   double d, double s);
double max_travel_time_to_end_no_stopping(double v_1, double v_2, double v_m,
                                          double a, double d, double s,
                                          double x);

double max_travel_time_from_start_stopping_allowed(double v_1, double v_2,
                                                   double a, double d, double s,
                                                   double x);
double max_travel_time_stopping_allowed(double v_1, double v_2, double a,
                                        double d, double s);
double max_travel_time_to_end_stopping_allowed(double v_1, double v_2, double a,
                                               double d, double s, double x);

double max_travel_time_from_start(double v_1, double v_2, double v_m, double a,
                                  double d, double s, double x,
                                  bool stopping_allowed);
double max_travel_time(double v_1, double v_2, double v_m, double a, double d,
                       double s, bool stopping_allowed);
double max_travel_time_to_end(double v_1, double v_2, double v_m, double a,
                              double d, double s, double x,
                              bool stopping_allowed);

double min_time_to_push_ma_forward(double v_0, double a, double d, double s);
double min_time_to_push_ma_backward(double v_0, double a, double d, double s);
double min_time_to_push_ma_fully_backward(double v_0, double a, double d);
double min_time_from_front_to_ma_point(double v_1, double v_2, double v_m,
                                       double a, double d, double s,
                                       double obd);
double min_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double obd);
double max_time_from_front_to_ma_point_no_stopping(double v_1, double v_2,
                                                   double v_m, double a,
                                                   double d, double s,
                                                   double obd);
double max_time_from_front_to_ma_point_stopping_allowed(double v_1, double v_2,
                                                        double a, double d,
                                                        double s, double obd);
double max_time_from_front_to_ma_point(double v_1, double v_2, double v_m,
                                       double a, double d, double s, double obd,
                                       bool stopping_allowed);
double max_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double obd);

enum class MATimingStrategy { ExtremeProfiles = 0 };

double min_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd,
    MATimingStrategy strategy = MATimingStrategy::ExtremeProfiles);
double max_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd,
    MATimingStrategy strategy = MATimingStrategy::ExtremeProfiles);

double maximal_line_speed(double v_1, double v_2, double v_max, double a,
                          double d, double s);
double minimal_line_speed(double v_1, double v_2, double v_min, double a,
                          double d, double s);
double time_on_edge(double v_1, double v_2, double v_line, double a, double d,
                    double s);
double pos_on_edge_at_time(double v_1, double v_2, double v_line, double a,
                           double d, double s, double t);
double vel_on_edge_at_time(double v_1, double v_2, double v_line, double a,
                           double d, double s, double t);
double get_line_speed(double v_1, double v_2, double v_min, double v_max,
                      double a, double d, double s, double t);
} // namespace cda_rail
