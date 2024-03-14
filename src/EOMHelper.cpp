#include "EOMHelper.hpp"

#include "CustomExceptions.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

double cda_rail::min_travel_time_from_start(double v_1, double v_2, double v_m,
                                            double a, double d, double s,
                                            double x) {
  check_consistency_of_eom_input(v_1, v_2, a, d, s, x);

  const auto s_points =
      get_min_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d, s);
  const auto& s_1 = s_points.first;
  const auto& s_2 = s_points.second;

  assert(s_2 >= s_1);

  // Accelerating part
  const double x_1 = std::min(x, s_1);

  // Time spent until x_1 is (sqrt(2*a*x_1+v_1^2)-v_1)/a
  // Stable version: (2*x_1)/(sqrt(2*a*x_1+v_1^2)+v_1)
  const double t_1 =
      x_1 == 0 ? 0 : (2 * x_1) / (std::sqrt(2 * a * x_1 + v_1 * v_1) + v_1);

  // Terminal velocity
  const double v_t_squared =
      v_1 * v_1 + 2 * a * s_1;               // Maximal velocity reached
  const double v_t = std::sqrt(v_t_squared); // = v_m if s_2 > s_1

  // Constant speed part
  const double x_2 = std::min(std::max(x - s_1, 0.0), s_2 - s_1);
  const double t_2 = x_2 == 0 ? 0 : x_2 / v_t;

  // Decelerating part
  const double x_3 = std::min(std::max(x - s_2, 0.0), s - s_2);
  // Time spent until x_3 is (2*x_3)/(sqrt(v_m^2-2*d*x_3)+v_m)
  const double t_3 =
      x_3 == 0 ? 0 : (2 * x_3) / (std::sqrt(v_t_squared - 2 * d * x_3) + v_t);

  return t_1 + t_2 + t_3;
}

double cda_rail::min_travel_time(double v_1, double v_2, double v_m, double a,
                                 double d, double s) {
  return min_travel_time_from_start(v_1, v_2, v_m, a, d, s, s);
}

bool cda_rail::possible_by_eom(double v_1, double v_2, double a, double d,
                               double s) {
  return v_1 <= v_2 ? (v_2 + v_1) * (v_2 - v_1) <= 2 * a * s
                    : (v_1 + v_2) * (v_1 - v_2) <= 2 * d * s;
}

double cda_rail::max_travel_time_from_start_no_stopping(double v_1, double v_2,
                                                        double v_m, double a,
                                                        double d, double s,
                                                        double x) {
  // v_m is minimal speed in this case

  check_consistency_of_eom_input(v_1, v_2, a, d, s, x);

  const bool v_1_below_minimal_speed = v_1 < v_m;
  const bool v_2_below_minimal_speed = v_2 < v_m;

  const auto s_points =
      get_max_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d, s);
  const auto& s_1 = s_points.first;
  const auto& s_2 = s_points.second;

  // Acceleration/Decelation to s_1
  const double x_1 = std::min(x, s_1);

  // Time spent until x_1 is (sqrt(v_1^2-2*d*x_1)-v_1)/d
  // Stable version: (2*x_1)/(sqrt(v_1^2-2*d*x_1)+v_1)
  const double t_1 =
      x_1 == 0 ? 0
               : (2 * x_1) /
                     (std::sqrt(v_1 * v_1 +
                                2 * (v_1_below_minimal_speed ? a : -d) * x_1) +
                      v_1);

  const double v_t_squared =
      v_1 * v_1 +
      2 * (v_1_below_minimal_speed ? a : -d) * s_1; // Minimal velocity reached
  const double v_t = std::sqrt(v_t_squared);        // = v_m if s_2 > s_1

  const double x_2 = std::min(std::max(x - s_1, 0.0), s_2 - s_1);
  const double t_2 = x_2 == 0 ? 0 : x_2 / v_t; // constant

  const double x_3 = std::min(std::max(x - s_2, 0.0), s - s_2);
  // Time spent until x_3 is (sqrt(2*a*x_3+v_m^2)-v_m)/a
  // Stable version: (2*x_3)/(sqrt(2*a*x_3+v_m^2)+v_m)
  const double t_3 =
      x_3 == 0 ? 0
               : (2 * x_3) /
                     (std::sqrt(v_t_squared +
                                2 * (v_2_below_minimal_speed ? -d : a) * x_3) +
                      v_t);
  return t_1 + t_2 + t_3;
}

double cda_rail::max_travel_time_no_stopping(double v_1, double v_2, double v_m,
                                             double a, double d, double s) {
  return max_travel_time_from_start_no_stopping(v_1, v_2, v_m, a, d, s, s);
}

double cda_rail::max_travel_time_from_start_stopping_allowed(
    double v_1, double v_2, double a, double d, double s, double x) {
  check_consistency_of_eom_input(v_1, v_2, a, d, s, x);

  const auto s_points =
      get_max_travel_time_acceleration_change_points(v_1, v_2, 0, a, d, s);
  const auto& s_1 = s_points.first;
  const auto& s_2 = s_points.second;

  const double bd = v_1 * v_1 / (2 * d); // Distance to stop

  if (bd <= s_1) {
    if (x >= s_1) {
      // Infinite, because train could have stopped
      return std::numeric_limits<double>::infinity();
    }
  }

  // Train cannot stop, hence same as max_travel_time_from_start_no_stopping
  return max_travel_time_from_start_no_stopping(v_1, v_2, 0, a, d, s, x);
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

double cda_rail::max_travel_time(double v_1, double v_2, double v_m, double a,
                                 double d, double s, bool stopping_allowed) {
  return max_travel_time_from_start(v_1, v_2, v_m, a, d, s, s,
                                    stopping_allowed);
}

void cda_rail::check_consistency_of_eom_input(double v_1, double v_2, double a,
                                              double d, double s, double x) {
  if (v_1 < 0 || v_2 < 0 || a < 0 || d < 0 || s < 0 || x < 0) {
    throw exceptions::ConsistencyException(
        "All input values must be non-negative.");
  }

  if (x > s) {
    throw exceptions::ConsistencyException(
        "x must be less than or equal to s.");
  }

  if (!possible_by_eom(v_1, v_2, a, d, s)) {
    throw exceptions::ConsistencyException(
        "Travel time not possible by equations of motion.");
  }
}

double cda_rail::min_travel_time_to_end(double v_1, double v_2, double v_m,
                                        double a, double d, double s,
                                        double x) {
  // Same as minimal travel time going the reverse direction with acceleration
  // and deceleration swapped
  // NOLINTNEXTLINE(readability-suspicious-call-argument)
  return min_travel_time_from_start(v_2, v_1, v_m, d, a, s, s - x);
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

double cda_rail::max_travel_time_to_end(double v_1, double v_2, double v_m,
                                        double a, double d, double s, double x,
                                        bool stopping_allowed) {
  return stopping_allowed
             ? max_travel_time_to_end_stopping_allowed(v_1, v_2, a, d, s, x)
             : max_travel_time_to_end_no_stopping(v_1, v_2, v_m, a, d, s, x);
}

double cda_rail::min_time_to_push_ma_forward(double v_0, double a, double d,
                                             double s) {
  // How much time does a train need to move its moving authority forward by s
  // given initial speed v_0 and acceleration a and deceleration d

  // Assert that v_0 >= 0, a >= 0, d > 0, s > 0
  if (v_0 < 0 || a < 0 || d <= 0 || s <= 0) {
    throw exceptions::InvalidInputException(
        "We need v_0 >= 0, a >= 0, d > 0, s > 0");
  }

  // ma(t) = v_0*t + 0.5*a*t^2 + (v_0+a*t)^2/(2d) != s + v_0^2/(2d)
  // -> t = (...) = 2*d*s / (sqrt(2*(a+d)*a*d*s+(a+d)^2*v^2) + (a+d)*v)
  return 2 * d * s /
         (std::sqrt(2 * (a + d) * a * d * s + (a + d) * (a + d) * v_0 * v_0) +
          (a + d) * v_0);
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

  const double s_1 =
      (v_m + v_1) * (v_m - v_1) / (2 * a); // Distance to reach maximal speed

  const double s_2 = s - ((v_m + v_2) * (v_m - v_2) / (2 * d));

  if (s_2 >= s_1) {
    return {s_1, s_2};
  }

  const double y = (2 * d * s + (v_2 + v_1) * (v_2 - v_1)) / (2 * (a + d));
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

  const double s_1 =
      (v_1 + v_m) * (v_1 - v_m) /
      (2 *
       (v_1_below_minimal_speed ? -a : d)); // Distance to reach minimal speed

  const double s_2 = s - ((v_2 + v_m) * (v_2 - v_m) /
                          (2 * (v_2_below_minimal_speed ? -d : a)));

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

  const double y = (2 * a * s + (v_1 + v_2) * (v_1 - v_2)) /
                   (2 * (a + d)); // Distance at which accelerating starts if
  // minimal speed is not reached

  return {y, y};
}
