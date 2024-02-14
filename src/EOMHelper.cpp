#include "EOMHelper.hpp"

#include "CustomExceptions.hpp"

#include <algorithm>
#include <cmath>

double cda_rail::min_travel_time(double v_1, double v_2, double v_m, double a,
                                 double d, double s, double x) {
  if (!possible_by_eom(v_1, v_2, a, d, s)) {
    throw exceptions::ConsistencyException(
        "Travel time not possible by equations of motion.");
  }

  const double s_1 =
      (v_m + v_1) * (v_m - v_1) / (2 * a); // Distance to reach maximal speed

  if (const double s_2 = s - ((v_m + v_2) * (v_m - v_2) / (2 * d));
      s_2 >= s_1) {
    // Accelerate to maximal speed. Keep constant until braking in the last
    // moment.

    const double x_1 = std::min(x, s_1);

    // Time spent until x_1 is (sqrt(2*a*x_1+v_1^2)-v_1)/a
    // Stable version: (2*x_1)/(sqrt(2*a*x_1+v_1^2)+v_1)
    const double t_1 = (2 * x_1) / (std::sqrt(2 * a * x_1 + v_1 * v_1) + v_1);

    const double x_2 = std::min(std::max(x - s_1, 0.0), s_2 - s_1);
    const double t_2 = x_2 / v_m; // constant

    const double x_3 = std::min(std::max(x - s_2, 0.0), s - s_2);
    // Time spent until x_3 is (2*x_3)/(sqrt(v_m^2-2*d*x_3)+v_m)
    const double t_3 = (2 * x_3) / (std::sqrt(v_m * v_m - 2 * d * x_3) + v_m);
    return t_1 + t_2 + t_3;
  }

  // Accelerate until braking is necessary to reach v_2

  const double y =
      (2 * d * s + (v_2 + v_1) * (v_2 - v_1)) /
      (2 *
       (a +
        d)); // Distance at which braking starts if maximal speed is not reached

  const double x_1 = std::min(x, y); // Accelerating part
  const double t_1 = (2 * x_1) / (std::sqrt(2 * a * x_1 + v_1 * v_1) + v_1);

  const double v_t_squared =
      v_1 * v_1 + 2 * a * y; // Maximal velocity reached before braking

  const double x_2 =
      std::min(std::max(x - y, 0.0), s - y); // Distance in braking part
  const double t_2 = (2 * x_2) / (std::sqrt(v_t_squared - 2 * d * x_2) +
                                  std::sqrt(v_t_squared));

  return t_1 + t_2;
}

double cda_rail::min_travel_time(double v_1, double v_2, double v_m, double a,
                                 double d, double s) {
  return min_travel_time(v_1, v_2, v_m, a, d, s, s);
}

bool cda_rail::possible_by_eom(double v_1, double v_2, double a, double d,
                               double s) {
  return v_1 <= v_2 ? (v_2 + v_1) * (v_2 - v_1) <= 2 * a * s
                    : (v_1 + v_2) * (v_1 - v_2) <= 2 * d * s;
}
