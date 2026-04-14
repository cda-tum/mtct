#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"

#include <concepts>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Logger.h>
#include <plog/Severity.h>

namespace cda_rail {

// Comparison Helper

/**
 * @brief Compares two floating-point values using a machine-epsilon-scaled
 *        tolerance.
 * @tparam T Floating-point type of the compared values.
 * @param a First value.
 * @param b Second value.
 * @param factor Scaling factor applied to `std::numeric_limits<T>::epsilon()`.
 * @return `true` if `|a - b| < factor * epsilon`, otherwise `false`.
 * @pre `factor >= 0` for a meaningful tolerance interpretation.
 * @throws cda_rail::exceptions::InvalidInputException If @p factor is
 * negative.
 */
template <std::floating_point T>
[[nodiscard]] bool approx_equal(T const a, T const b, T const factor = 10) {
  cda_rail::exceptions::throw_if_negative(factor, "factor");
  return std::abs(a - b) < factor * std::numeric_limits<T>::epsilon();
}

// Rounding Functions

/**
 * @brief Replaces a small-magnitude value with zero by modifying the input
 *        reference in place.
 * @param val Value to normalize in place.
 * @param tol Absolute tolerance below which @p val is set to zero.
 * @pre `tol >= 0` for a meaningful threshold.
 * @throws cda_rail::exceptions::InvalidInputException If @p tol is negative.
 * If `std::abs(val) < tol`, this function overwrites @p val with `0`;
 * otherwise, it leaves @p val unchanged.
 */
static void round_small_numbers_to_zero_inplace(double&      val,
                                                double const tol = EPS) {
  cda_rail::exceptions::throw_if_negative(tol, "tol");
  if (std::abs(val) < tol) {
    val = 0;
  }
}

/**
 * @brief Rounds a value to the nearest multiple of a given tolerance.
 * @param value Value to round.
 * @param tolerance Positive rounding step, for example `1e-5`.
 * @return Rounded value.
 * @pre `tolerance > 0`.
 * @throws cda_rail::exceptions::InvalidInputException If @p tolerance is not
 * strictly positive.
 * The returned value is a multiple of @p tolerance up to floating-point
 * round-off.
 */
[[nodiscard]] static double round_to_given_tolerance(double const value,
                                                     double const tolerance) {
  cda_rail::exceptions::throw_if_non_positive(tolerance, "tolerance");
  const auto factor = std::round(1.0 / tolerance);
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

// String Helper

/**
 * @brief Concatenates multiple string views into a single string.
 * @param parts String fragments to concatenate in the given order.
 * @return String containing all fragments from @p parts appended in order.
 * The length of the returned string equals the sum of the fragment lengths.
 */
[[nodiscard]] std::string
concatenate_string_views(std::initializer_list<std::string_view> parts);

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

} // namespace cda_rail
