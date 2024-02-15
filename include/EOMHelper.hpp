#pragma once

// EOM = Equations of Motion

namespace cda_rail {
bool   possible_by_eom(double v_1, double v_2, double a, double d, double s);
double min_travel_time_from_start(double v_1, double v_2, double v_m, double a,
                                  double d, double s, double x);
double min_travel_time(double v_1, double v_2, double v_m, double a, double d,
                       double s);

double max_travel_time_from_start_no_stopping(double v_1, double v_2,
                                              double v_m, double a, double d,
                                              double s, double x);
double max_travel_time_no_stopping(double v_1, double v_2, double v_m, double a,
                                   double d, double s);
} // namespace cda_rail
