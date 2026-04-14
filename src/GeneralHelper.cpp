#include "GeneralHelper.hpp"

#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Logger.h>
#include <plog/Severity.h>

// String Helper

std::string cda_rail::concatenate_string_views(
    std::initializer_list<std::string_view> parts) {
  const std::size_t total_size = std::ranges::fold_left(
      parts, std::size_t{0},
      [](std::size_t const sum, std::string_view const part) {
        return sum + part.size();
      });

  std::string result{};
  result.reserve(total_size);

  std::ranges::for_each(parts, [&result](std::string_view const part) {
    result.append(part.data(), part.size());
  });

  return result;
}

// Combinatorial Helper

std::vector<cda_rail::index_set>
cda_rail::subsets_of_size_k_indices(size_t const n, size_t const k) {
  // Throw an error if k > n
  if (k > n) {
    throw std::invalid_argument("k must be between 0 and n");
  }

  // If k = 0, return the empty set
  if (k == 0) {
    return {}; // would not work with std::unordered_set instead of std::vector
  }

  // If k = n, return the set {0, 1, ..., n-1}
  if (k == n) {
    std::vector<cda_rail::index_set> subsets;
    subsets.emplace_back();
    for (size_t i = 0; i < n; i++) {
      subsets.at(0).insert(i);
    }
    return subsets;
  }

  // Otherwise use recursion
  auto subsets_without_last_element = subsets_of_size_k_indices(n - 1, k);
  auto subsets_with_last_element    = subsets_of_size_k_indices(n - 1, k - 1);
  if (subsets_with_last_element.empty()) {
    subsets_with_last_element.emplace_back();
  }
  for (auto& subset : subsets_with_last_element) {
    subset.insert(n - 1);
  }
  subsets_without_last_element.insert(subsets_without_last_element.end(),
                                      subsets_with_last_element.begin(),
                                      subsets_with_last_element.end());
  return subsets_without_last_element;
}

std::vector<std::pair<size_t, size_t>>
cda_rail::subsets_of_size_2_indices(size_t n) {
  auto subsets = subsets_of_size_k_indices(n, 2);

  // convert index_set to pair
  std::vector<std::pair<size_t, size_t>> subsets_of_pairs;
  subsets_of_pairs.reserve(subsets.size());
  for (auto& subset : subsets) {
    assert(subset.size() == 2);
    auto it = subset.begin();
    subsets_of_pairs.emplace_back(*it, *(++it));
  }
  return subsets_of_pairs;
}

// Output Helper

void cda_rail::initialize_plog(bool const debugInput,
                               bool const overwriteSeverity) {
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }
  if (overwriteSeverity || plog::get()->getMaxSeverity() <= plog::info) {
    plog::get()->setMaxSeverity(debugInput ? plog::debug : plog::info);
  }
};

bool cda_rail::is_directory_and_create(const std::filesystem::path& p) {
  // If p is current directory, return true
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
};
