#include "GeneralHelper.hpp"

#include "Definitions.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <iterator>
#include <numeric>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Logger.h>
#include <plog/Severity.h>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

/**
 * @brief Generates all k-sized index combinations from [0, n) in lexicographic order.
 *
 * @param n Upper bound (exclusive) for indices.
 * @param k Number of indices in each combination.
 * @return A vector of all k-combinations of [0, n).
 * @throws std::invalid_argument if k > n.
 */

std::vector<cda_rail::index_vector>
cda_rail::subsets_of_size_k_indices(size_t const n, size_t const k) {
  if (k > n) {
    throw std::invalid_argument("k must be between 0 and n");
  }
  if (k == 0) {
    return {}; // would not work with std::unordered_set instead of std::vector
  }

  // indices always hold the current combination in ascending order.
  // It starts at {0, 1, ..., k-1} and is stepped through all C(n,k)
  // k-combinations of {0, ..., n-1} in lexicographic order.
  std::vector<cda_rail::index_vector> result;
  std::vector<size_t>                 indices(k);
  // Do not use std::ranges::iota due to macOS incompatibility
  // NOLINTNEXTLINE
  std::iota(indices.begin(), indices.end(),
            0UZ); // first combination: {0, 1, ..., k-1}

  // Advance indices to the lexicographically next k-combination.
  // Returns true if the next combination exists, false if all are exhausted.
  const auto advance_to_next_combination = [&indices, n, k]() -> bool {
    // Find the rightmost position whose index can still be incremented.
    // Position i is at its maximum when indices[i] == n - k + i.
    auto i = static_cast<ptrdiff_t>(k) - 1;
    while (i >= 0 && indices.at(i) == n - k + static_cast<size_t>(i)) {
      --i;
    }
    if (i < 0) {
      return false; // all combinations exhausted
    }
    // Increment that position and reset everything to its right.
    ++indices.at(i);
    // Do not use std::ranges::iota due to macOS incompatibility
    // NOLINTNEXTLINE
    std::iota(std::next(indices.begin(), i + 1), indices.end(),
              indices.at(i) + 1);
    return true;
  };

  bool has_next_combination = true;
  while (has_next_combination) {
    result.emplace_back(indices.cbegin(), indices.cend());
    has_next_combination = advance_to_next_combination();
  }

  return result;
}

/**
 * @brief Generates all unordered pairs (i, j) with 0 ≤ i < j < n.
 *
 * @param n Upper bound (exclusive) for indices.
 * @return Vector containing all pairs (i, j) where 0 ≤ i < j < n.
 * @throws std::invalid_argument if n < 2.
 */
std::vector<std::pair<size_t, size_t>>
cda_rail::subsets_of_size_2_indices(size_t const n) {
  if (n < 2) {
    throw std::invalid_argument("n must be at least 2");
  }
  // Direct double loop: avoids calling the general function and a second
  // conversion pass.
  std::vector<std::pair<size_t, size_t>> result;
  result.reserve(n * (n - 1) / 2);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      result.emplace_back(i, j);
    }
  }
  return result;
}

/**
 * @brief Initializes plog logging and configures the maximum severity threshold.
 *
 * @param debugInput If `true`, sets the maximum severity to debug; if `false`, sets it to info.
 * @param overwriteSeverity If `true`, always updates the severity threshold. If `false`, updates
 *                           only if the current severity level is at or below info.
 */

void cda_rail::initialize_plog(bool const debugInput,
                               bool const overwriteSeverity) {
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }
  if (overwriteSeverity || plog::get()->getMaxSeverity() <= plog::info) {
    plog::get()->setMaxSeverity(debugInput ? plog::debug : plog::info);
  }
}

/**
 * @brief Creates the directory at the given path if it does not exist.
 *
 * @param p Path to the directory.
 * @return `true` if the path is a directory or is empty, `false` if creation failed
 *         or the path exists but is not a directory.
 */
bool cda_rail::is_directory_and_create(const std::filesystem::path& p) {
  // If p is empty, treat as current directory — always valid.
  if (p.empty()) {
    return true;
  }
  if (!std::filesystem::exists(p)) {
    std::error_code error_code;
    std::filesystem::create_directories(p, error_code);
    if (error_code) {
      return false;
    }
  }
  return std::filesystem::is_directory(p);
}

/**
 * @brief Computes the ReLU activation.
 *
 * @param val The input value.
 * @return double The input value clamped to a minimum of zero.
 */
double cda_rail::relu(double const val) { return std::max(0.0, val); }
