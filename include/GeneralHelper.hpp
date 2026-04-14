#pragma once

#include "Definitions.hpp"

#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Logger.h>
#include <plog/Severity.h>

namespace cda_rail {

// Comparison Helper
// Template type T
template <typename T>
static bool approx_equal(T const a, T const b, T const factor = 10) {
  const auto eps = factor * std::numeric_limits<T>::epsilon();
  return a - b < eps && b - a < eps;
}

// Rounding Functions

static void round_small_numbers_to_zero_inplace(double&      val,
                                                double const tol = EPS) {
  if (std::abs(val) < tol) {
    val = 0;
  }
};

static double round_to_given_tolerance(double const value,
                                       double const tolerance) {
  /**
   * Round value to the given the tolerance, e.g., 1e-5.
   * @param value Value to be rounded
   * @param tolerance Tolerance
   *
   * @return Rounded value
   */

  const auto factor = std::round(1 / tolerance);
  return std::round(value * factor) / factor;
}

// Combinatorial Helper

/**
 * Returns a vector of all subsets of size k of the set {0, 1, ..., n-1} as
 * pairs of indices. The order is not important, i.e. {0, 1} and {1, 0} are
 * considered the same.
 *
 * @param n Size of the set
 * @param k Size of the subsets
 * @return Vector of all subsets of size k of the set {0, 1, ..., n-1} as
 * pairs of indices
 */
std::vector<cda_rail::index_set> subsets_of_size_k_indices(size_t n, size_t k);

std::vector<std::pair<size_t, size_t>> subsets_of_size_2_indices(size_t n);

// String Helper

std::string
concatenate_string_views(std::initializer_list<std::string_view> parts);

// Debugging / Output Helper

void initialize_plog(bool debug_input, bool overwrite_severity = false);

/**
 * Checks if a directory exists and creates it if it doesn't.
 * Returns true if the directory exists or was created successfully.
 * Returns false if the directory could not be created, probably because the
 * path is not a directory.
 *
 * @param p Path to the directory
 */
bool is_directory_and_create(const std::filesystem::path& p);

} // namespace cda_rail
