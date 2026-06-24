#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"

#include <chrono>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <utility>
#include <vector>

namespace cda_rail {

// Comparison Helper

/**
 * @brief Determines if two floating-point values are approximately equal using
 * a machine-epsilon-scaled tolerance.
 * @tparam T Floating-point type of the compared values.
 * @param a First value.
 * @param b Second value.
 * @param factor Scaling factor applied to `std::numeric_limits<T>::epsilon()`.
 * @return `true` if `|a - b| < factor * epsilon`, otherwise `false`.
 * @pre `factor >= 0` for a meaningful tolerance interpretation.
 * @throws cda_rail::exceptions::InvalidInputException If @p the factor is
 * negative.
 */
template <std::floating_point T>
[[nodiscard]] bool approx_equal(T const a, T const b, T const factor = 10) {
  cda_rail::exceptions::throw_if_negative(factor, "factor");
  return std::abs(a - b) < factor * std::numeric_limits<T>::epsilon();
}

// Rounding Functions

/**
 * @brief Sets a value to zero if its absolute magnitude is less than a
 * tolerance threshold.
 *
 * If the absolute value of @p val is less than @p tol, @p val is set to zero;
 * otherwise, @p val remains unchanged.
 *
 * @param val Reference to the value to normalize.
 * @param tol Absolute tolerance threshold.
 * @throws cda_rail::exceptions::InvalidInputException If @p tol is negative.
 */
inline void round_small_numbers_to_zero_inplace(double&      val,
                                                double const tol = EPS) {
  cda_rail::exceptions::throw_if_negative(tol, "tol");
  if (std::abs(val) < tol) {
    val = 0;
  }
}

/**
 * @brief Rounds a value to the nearest multiple of a given tolerance.
 *
 * @param value Value to round.
 * @param tolerance Positive rounding step.
 * @return Value rounded to the nearest multiple of @p tolerance, up to
 * floating-point round-off.
 * @throws cda_rail::exceptions::InvalidInputException If @p tolerance is not
 * strictly positive.
 */
[[nodiscard]] inline double round_to_given_tolerance(double const value,
                                                     double const tolerance) {
  cda_rail::exceptions::throw_if_non_positive(tolerance, "tolerance");
  const auto factor = 1.0 / tolerance;
  return std::round(value * factor) / factor;
}

// Combinatorial Helper

/**
 * @brief Generates all subsets of size @p k of the index set
 *        `{0, 1, ..., n - 1}`.
 * @param n Size of the underlying index set.
 * @param k Required subset size.
 * @return Vector containing all `k`-element index subsets.
 * @pre `k <= n`.
 * @throws std::invalid_argument If `k > n`.
 * Returns an empty vector if `k == 0`.
 */
[[nodiscard]] std::vector<cda_rail::index_vector>
subsets_of_size_k_indices(size_t n, size_t k);

/**
 * @brief Generates all unordered index pairs from `{0, 1, ..., n - 1}`.
 * @param n Size of the underlying index set.
 * @return Vector containing all pairs `(i, j)` with `0 <= i < j < n`.
 * @pre `n >= 2`.
 * @throws std::invalid_argument If `n < 2`.
 */
[[nodiscard]] std::vector<std::pair<size_t, size_t>>
subsets_of_size_2_indices(size_t n);

// Debugging / Output Helper

/**
 * @brief Initializes the global plog console logger and adjusts its severity.
 * @param debug_input If `true`, configures debug logging; otherwise configures
 *                    info logging when an update is applied.
 * @param overwrite_severity If `true`, updates the maximum severity even when
 *                           a logger already exists.
 * After the call, a global plog logger instance exists.
 */
void initialize_plog(bool debug_input, bool overwrite_severity = false);

/**
 * @brief Ensures that a filesystem path exists as a directory.
 * @param p Path to the directory.
 * @return `true` if @p p is empty, already denotes a directory, or the
 *         directory was created successfully; otherwise `false`.
 * If the function returns `true` and @p p is non-empty, @p p denotes an
 * existing directory.
 */
[[nodiscard]] bool is_directory_and_create(const std::filesystem::path& p);

/**
 * @brief Returns the positive part of a scalar.
 *
 * @param val Input value.
 * @return `val` if it is positive, otherwise `0`.
 */
[[nodiscard]] double relu(double val);

template <typename Clock, typename Duration1, typename Duration2>
[[nodiscard]] inline double get_time_difference_in_seconds(
    std::chrono::time_point<Clock, Duration1> const& start,
    std::chrono::time_point<Clock, Duration2> const& end) {
  return static_cast<double>(
             std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                 .count()) /
         1000.0;
}

/**
 * @brief return last time step before t
 *
 * @param t Time t
 * @param dt Timestep
 * @param t_inclusive Can t itself be returned?
 * @return max (k * dt s.th. k*dt <= t) if t_inclusive is true, otherwise max (k
 * * dt s.th. k*dt < t)
 */
[[nodiscard]] double get_last_time_step_before(double t, double dt,
                                               bool t_inclusive);
/**
 * @brief return first time step after t
 *
 * @param t Time t
 * @param dt Timestep
 * @param t_inclusive Can t itself be returned?
 * @return min (k * dt s.th. k*dt >= t) if t_inclusive is true, otherwise min (k
 * * dt s.th. k*dt > t)
 */
[[nodiscard]] double get_first_time_step_after(double t, double dt,
                                               bool t_inclusive);

} // namespace cda_rail
