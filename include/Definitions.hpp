#pragma once
#include <filesystem>
#include <vector>

namespace cda_rail {
// Constants
const double INF           = std::numeric_limits<double>::max() / 3;
const double ABS_PWL_ERROR = 10;

// Constants for vertex type
enum class VertexType { NoBorder = 0, VSS = 1, TTD = 2, NoBorderVss = 3 };
enum class SeparationType { UNIFORM, CHEBYCHEV };

// Helper functions

static bool is_directory_and_create(const std::filesystem::path& p) {
  /**
   * Checks if a directory exists and creates it if it doesn't.
   * Returns true if the directory exists or was created successfully.
   * Returns false if the directory could not be created, probably because the
   * path is not a directory.
   *
   * @param p Path to the directory
   */

  if (!std::filesystem::exists(p)) {
    std::error_code error_code;
    std::filesystem::create_directories(p, error_code);
    if (error_code) {
      return false;
    }
  }
  return std::filesystem::is_directory(p);
};

static std::vector<std::vector<size_t>> subsets_of_size_k_indices(size_t n,
                                                                  size_t k) {
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

  // Throw an error if k > n
  if (k > n) {
    throw std::invalid_argument("k must be between 0 and n");
  }

  // If k = 0, return the empty set
  if (k == 0) {
    return std::vector<std::vector<size_t>>();
  }

  // If k = n, return the set {0, 1, ..., n-1}
  if (k == n) {
    std::vector<std::vector<size_t>> subsets;
    subsets.emplace_back();
    for (size_t i = 0; i < n; i++) {
      subsets[0].emplace_back(i);
    }
    return subsets;
  }

  // Otherwise use recursion
  auto subsets_without_n = subsets_of_size_k_indices(n - 1, k);
  auto subsets_with_n    = subsets_of_size_k_indices(n - 1, k - 1);
  if (subsets_with_n.empty()) {
    subsets_with_n.emplace_back();
  }
  for (auto& subset : subsets_with_n) {
    subset.emplace_back(n - 1);
  }
  subsets_without_n.insert(subsets_without_n.end(), subsets_with_n.begin(),
                           subsets_with_n.end());
  return subsets_without_n;
}

static std::vector<std::pair<size_t, size_t>>
subsets_of_size_2_indices(size_t n) {
  auto subsets = subsets_of_size_k_indices(n, 2);
  std::vector<std::pair<size_t, size_t>> subsets_of_pairs;
  for (auto& subset : subsets) {
    subsets_of_pairs.emplace_back(subset[0], subset[1]);
  }
  return subsets_of_pairs;
}
} // namespace cda_rail
