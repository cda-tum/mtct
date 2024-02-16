#pragma once

// EOM = Equations of Motion

namespace cda_rail {
bool possible_by_eom(double v_1, double v_2, double a, double d, double s);
void check_consistency_of_eom_input(double v_1, double v_2, double a, double d,
                                    double s, double x);

double min_travel_time_from_start(double v_1, double v_2, double v_m, double a,
                                  double d, double s, double x);
double min_travel_time(double v_1, double v_2, double v_m, double a, double d,
                       double s);

double max_travel_time_from_start_no_stopping(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double x);
double max_travel_time_no_stopping(double v_1, double v_2, double v_m, double a,
                                   double d, double s);

double max_travel_time_from_start_stopping_allowed(double v_1, double v_2,
                                                   double a, double d, double s,
                                                   double x);
double max_travel_time_stopping_allowed(double v_1, double v_2, double a,
                                        double d, double s);

double max_travel_time_from_start(double v_1, double v_2, double v_m, double a,
                                  double d, double s, double x,
                                  bool stopping_allowed);
double max_travel_time(double v_1, double v_2, double v_m, double a, double d,
                       double s, bool stopping_allowed);
} // namespace cda_rail
