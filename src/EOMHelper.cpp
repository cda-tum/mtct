#include "EOMHelper.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <string>

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
      x_3 == 0
          ? 0
          : (2 * x_3) /
                (std::sqrt(std::max(0.0, v_t_squared - 2 * d * x_3)) + v_t);

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
      x_1 == 0
          ? 0
          : (2 * x_1) /
                (std::sqrt(std::max(
                     0.0, v_1 * v_1 +
                              2 * (v_1_below_minimal_speed ? a : -d) * x_1)) +
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
      x_3 == 0
          ? 0
          : (2 * x_3) /
                (std::sqrt(std::max(
                     0.0, v_t_squared +
                              2 * (v_2_below_minimal_speed ? -d : a) * x_3)) +
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

void cda_rail::check_consistency_of_eom_input(double& v_1, double& v_2,
                                              double& a, double& d, double& s,
                                              double& x) {
  if (std::abs(v_1) < GRB_EPS) {
    v_1 = 0;
  }
  if (std::abs(v_2) < GRB_EPS) {
    v_2 = 0;
  }
  if (std::abs(a) < GRB_EPS) {
    a = 0;
  }
  if (std::abs(d) < GRB_EPS) {
    d = 0;
  }
  if (std::abs(s) < GRB_EPS) {
    s = 0;
  }
  if (std::abs(x) < GRB_EPS) {
    x = 0;
  }
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

  if (std::abs(v_0) < GRB_EPS) {
    v_0 = 0;
  }
  if (std::abs(a) < GRB_EPS) {
    a = 0;
  }
  if (d < 0 && d > -GRB_EPS) {
    d = 0;
  }
  if (std::abs(s) < GRB_EPS) {
    s = 0;
  }

  // Assert that v_0 >= 0, a >= 0, d > 0, s > 0
  if (v_0 < 0 || a < 0 || d <= 0 || s < 0) {
    throw exceptions::InvalidInputException(
        "We need v_0 >= 0, a >= 0, d > 0, s >= 0");
  }

  // ma(t) = v_0*t + 0.5*a*t^2 + (v_0+a*t)^2/(2d) != s + v_0^2/(2d)
  // -> t = (...) = 2*d*s / (sqrt(2*(a+d)*a*d*s+(a+d)^2*v^2) + (a+d)*v)
  return s == 0 ? 0
                : 2 * d * s /
                      (std::sqrt(2 * (a + d) * a * d * s +
                                 (a + d) * (a + d) * v_0 * v_0) +
                       (a + d) * v_0);
}

double cda_rail::min_time_to_push_ma_fully_backward(double v_0, double a,
                                                    double d) {
  // Simplified version if s is the full braking distance
  // solve for t : (v-a*t)^2/(2*b) = v * t - 0.5 * a * t^2
  // -> t = (v + (a v)/b - (sqrt(a + b) v)/sqrt(b))/(a + a^2/b)
  // -> t = ((-sqrt(b) + sqrt(a + b)) v)/(a sqrt(a + b))
  // -> t = v / (a + b + sqrt(b*(a+b)))
  // Again b is replaced with the deceleration

  if (std::abs(v_0) < GRB_EPS) {
    v_0 = 0;
  }
  if (std::abs(a) < GRB_EPS) {
    a = 0;
  }
  if (d < 0 && d > -GRB_EPS) {
    d = 0;
  }

  // Assert that v_0 >= 0, a >= 0, d > 0
  if (v_0 < 0 || a < 0 || d <= 0) {
    throw exceptions::InvalidInputException("We need v_0 >= 0, a >= 0, d > 0");
  }

  return v_0 / (a + d + std::sqrt(d * (a + d)));
}

double cda_rail::min_time_to_push_ma_backward(double v_0, double a, double d,
                                              double s) {
  if (std::abs(v_0) < GRB_EPS) {
    v_0 = 0;
  }
  if (std::abs(a) < GRB_EPS) {
    a = 0;
  }
  if (d < 0 && d > -GRB_EPS) {
    d = 0;
  }
  if (std::abs(s) < GRB_EPS) {
    s = 0;
  }

  // Assert that v_0 >= 0, a >= 0, d > 0, s > 0
  if (v_0 < 0 || a < 0 || d <= 0 || s < 0) {
    throw exceptions::InvalidInputException(
        "We need v_0 >= 0, a >= 0, d > 0, s >= 0");
  }

  // Assert that s <= v_0^2/(2d)
  if (s > v_0 * v_0 / (2 * d) + GRB_EPS) {
    throw exceptions::InvalidInputException(
        "s must be less than or equal to v_0^2/(2d)");
  }

  if (std::abs(v_0 * v_0 / (2 * d) - s) < GRB_EPS) {
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

  return 2 * d * s /
         ((a + d) * (v_0 + std::sqrt(v_0 * v_0 - 2 * a * d * s / (a + d))));
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

double cda_rail::min_time_from_front_to_ma_point(double v_1, double v_2,
                                                 double v_m, double a, double d,
                                                 double s, double obd) {
  const auto s_points =
      get_min_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d, s);
  const auto& s_1 = s_points.first;
  const auto& s_2 = s_points.second;

  if (std::abs(obd) < GRB_EPS) {
    obd = 0;
  }
  if (obd < 0) {
    throw exceptions::InvalidInputException(
        "obd must be greater than or equal 0.");
  }
  if (obd == 0) {
    return min_travel_time(v_1, v_2, v_m, a, d, s);
  }

  const double bd_1  = v_1 * v_1 / (2 * d); // Distance to stop
  const double bd_2  = v_2 * v_2 / (2 * d); // Distance to stop
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

double cda_rail::min_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                                        double v_m, double a,
                                                        double d, double s,
                                                        double obd) {
  return min_travel_time(v_1, v_2, v_m, a, d, s) -
         min_time_from_front_to_ma_point(v_1, v_2, v_m, a, d, s, obd);
}

double cda_rail::max_time_from_front_to_ma_point_no_stopping(
    double v_1, double v_2, double v_m, double a, double d, double s,
    double obd) {
  const auto s_points =
      get_max_travel_time_acceleration_change_points(v_1, v_2, v_m, a, d, s);
  const auto& s_1 = s_points.first;
  const auto& s_2 = s_points.second;

  if (std::abs(obd) < GRB_EPS) {
    obd = 0;
  }
  if (obd < 0) {
    throw exceptions::InvalidInputException(
        "obd must be greater than or equal 0.");
  }

  const bool v1_below_minimal_speed = v_1 < v_m;

  // Speed at changing point
  const double v_t_squared =
      (v_1 * v_1) + (2 * (v1_below_minimal_speed ? a : -d) * s_1);
  const double v_t = std::sqrt(v_t_squared);

  // Braking distances
  const double bd_1 = v_1 * v_1 / (2 * d);
  const double bd_2 = v_2 * v_2 / (2 * d);
  const double bd_t = v_t_squared / (2 * d);

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

double cda_rail::max_time_profile_from_rear_to_ma_point(double v_1, double v_2,
                                                        double v_m, double a,
                                                        double d, double s,
                                                        double obd) {
  return max_travel_time_no_stopping(v_1, v_2, v_m, a, d, s) -
         max_time_from_front_to_ma_point_no_stopping(v_1, v_2, v_m, a, d, s,
                                                     obd);
}

double cda_rail::max_time_from_front_to_ma_point_stopping_allowed(
    double v_1, double v_2, double a, double d, double s, double obd) {
  if (max_travel_time_stopping_allowed(v_1, v_2, a, d, s) >=
      std::numeric_limits<double>::infinity()) {
    return std::numeric_limits<double>::infinity();
  }
  return max_time_from_front_to_ma_point_no_stopping(v_1, v_2, 0, a, d, s, obd);
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

double cda_rail::min_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd, cda_rail::MATimingStrategy strategy) {
  if (strategy == MATimingStrategy::ExtremeProfiles) {
    const double t1 =
        max_time_profile_from_rear_to_ma_point(v_1, v_2, v_min, a, d, s, obd);
    const double t2 =
        min_time_profile_from_rear_to_ma_point(v_1, v_2, v_max, a, d, s, obd);
    return std::min(t1, t2);
  }
  throw exceptions::InvalidInputException("Invalid strategy.");
}

double cda_rail::max_time_from_rear_to_ma_point(
    double v_1, double v_2, double v_min, double v_max, double a, double d,
    double s, double obd, cda_rail::MATimingStrategy strategy) {
  if (strategy == MATimingStrategy::ExtremeProfiles) {
    const double t1 =
        max_time_profile_from_rear_to_ma_point(v_1, v_2, v_min, a, d, s, obd);
    const double t2 =
        min_time_profile_from_rear_to_ma_point(v_1, v_2, v_max, a, d, s, obd);
    return std::max(t1, t2);
  }
  throw exceptions::InvalidInputException("Invalid strategy.");
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

  // If any variable is within std::abs(GRB_EPS), set to 0
  if (std::abs(v_1) < GRB_EPS) {
    v_1 = 0;
  }
  if (std::abs(v_2) < GRB_EPS) {
    v_2 = 0;
  }
  if (std::abs(v_line) < GRB_EPS) {
    v_line = 0;
  }
  if (std::abs(a) < GRB_EPS) {
    a = 0;
  }
  if (std::abs(d) < GRB_EPS) {
    d = 0;
  }
  if (std::abs(s) < GRB_EPS) {
    s = 0;
  }

  // Assert that all variables are >= 0 and a, d, s, v_line are >= GRB_EPS
  if (v_1 < 0 || v_2 < 0 || v_line < GRB_EPS || a < GRB_EPS || d < GRB_EPS ||
      s < 0) {
    throw exceptions::InvalidInputException(
        "All input values must be non-negative, and a, d, v_line must be "
        "greater "
        "than 0.");
  }

  if (s == 0) {
    return 0;
  }

  // First segment: v_1 -> v_line
  const double a1 = v_line >= v_1 ? a : -d;
  const double s1 = (v_line * v_line - v_1 * v_1) / (2 * a1);
  const double t1 = (v_line - v_1) / a1;
  assert(t1 >= 0);

  // Last segment: v_line -> v_2
  const double a2 = v_2 >= v_line ? a : -d;
  const double s2 = (v_2 * v_2 - v_line * v_line) / (2 * a2);
  const double t2 = (v_2 - v_line) / a2;
  assert(t2 >= 0);

  // If s1 + s2 > s, this is not possible
  if (s1 + s2 - GRB_EPS > s) {
    throw exceptions::ConsistencyException(
        "Travel time not possible by equations of motion.");
  }

  if (std::abs(s - s1 - s2) < GRB_EPS) {
    return t1 + t2;
  }

  return t1 + t2 + ((s - s1 - s2) / v_line);
}

double cda_rail::maximal_line_speed(double v_1, double v_2, double v_max,
                                    double a, double d, double s) {
  const auto [s_1, s_2] =
      get_min_travel_time_acceleration_change_points(v_1, v_2, v_max, a, d, s);

  assert(s_2 >= s_1);

  // Terminal velocity
  const double v_t_squared =
      (v_1 * v_1) + (2 * a * s_1); // Maximal velocity reached
  return std::sqrt(v_t_squared);   // = v_m if s_2 > s_1
}

double cda_rail::minimal_line_speed(double v_1, double v_2, double v_min,
                                    double a, double d, double s) {
  const bool v_1_below_minimal_speed = v_1 < v_min;

  const auto [s_1, s_2] =
      get_max_travel_time_acceleration_change_points(v_1, v_2, v_min, a, d, s);

  (void)s_2; // unused

  const double v_t_squared =
      (v_1 * v_1) + (2 * (v_1_below_minimal_speed ? a : -d) *
                     s_1);       // Minimal velocity reached
  return std::sqrt(v_t_squared); // = v_m if s_2 > s_1
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

double cda_rail::pos_on_edge_at_time(double v_1, double v_2, double v_line,
                                     double a, double d, double s, double t) {
  const auto total_time = time_on_edge(v_1, v_2, v_line, a, d, s);
  if (std::abs(total_time) < GRB_EPS) {
    return 0;
  }
  if (total_time < 0) {
    throw exceptions::InvalidInputException("Total travel time is negative.");
  }
  if (t > total_time + GRB_EPS) {
    // Time exceeds total time
    const auto v_line_dv     = v_line - LINE_SPEED_ACCURACY;
    const auto total_time_dv = time_on_edge(v_1, v_2, v_line_dv, a, d, s);
    if (t > total_time_dv + GRB_EPS) {
      throw exceptions::InvalidInputException(
          "Time exceeds total travel time in cda_rail::pos_on_edge_at_time "
          "with t = " +
          std::to_string(t) +
          " and total_time_dv = " + std::to_string(total_time_dv));
    }
    return s;
  }
  if (std::abs(t - total_time) < GRB_EPS) {
    return s;
  }

  const auto a1 = v_line >= v_1 ? a : -d;
  const auto a2 = v_2 >= v_line ? a : -d;

  const auto t1 = (v_line - v_1) / a1;
  const auto t2 = total_time - ((v_2 - v_line) / a2);

  if (t <= t1) {
    return (v_1 * t) + (0.5 * a1 * t * t);
  }
  if (t <= t2) {
    return (v_1 * t1) + (0.5 * a1 * t1 * t1) + (v_line * (t - t1));
  }
  return s + (0.5 * a2 * (total_time - t) * (total_time - t)) -
         (v_2 * (total_time - t));
}

double cda_rail::vel_on_edge_at_time(double v_1, double v_2, double v_line,
                                     double a, double d, double s, double t) {
  const auto total_time = time_on_edge(v_1, v_2, v_line, a, d, s);
  if (std::abs(total_time) < GRB_EPS) {
    return v_1;
  }
  if (total_time < 0) {
    throw exceptions::InvalidInputException("Total travel time is negative.");
  }
  if (t > total_time + GRB_EPS) {
    const auto v_line_dv     = v_line - LINE_SPEED_ACCURACY;
    const auto total_time_dv = time_on_edge(v_1, v_2, v_line_dv, a, d, s);
    if (t > total_time_dv + GRB_EPS) {
      throw exceptions::InvalidInputException(
          "Time exceeds total travel time in cda_rail::vel_on_edge_at_time "
          "with t = " +
          std::to_string(t) +
          " and total_time_dv = " + std::to_string(total_time_dv));
    }
    return v_2;
  }
  if (std::abs(t - total_time) < GRB_EPS) {
    return v_2;
  }

  const auto a1 = v_line >= v_1 ? a : -d;
  const auto a2 = v_2 >= v_line ? a : -d;

  const auto t1 = (v_line - v_1) / a1;
  const auto t2 = total_time - ((v_2 - v_line) / a2);

  if (t <= t1) {
    return v_1 + (a1 * t);
  }
  if (t <= t2) {
    return v_line;
  }
  return v_2 - (a2 * (total_time - t));
}
