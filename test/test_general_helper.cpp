#include "GeneralHelper.hpp"

#include "gtest/gtest.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <ranges>
#include <set>

// NOLINTBEGIN(clang-diagnostic-unused-result)

namespace {

[[nodiscard]] size_t choose(size_t n, size_t k) {
  if (k > n) {
    return 0;
  }
  if (k > n - k) {
    k = n - k;
  }
  size_t result = 1;
  for (size_t i = 1; i <= k; ++i) {
    result = result * (n - k + i) / i;
  }
  return result;
}

} // namespace

TEST(GeneralHelper, ApproxEqual) {
  constexpr auto eps = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(cda_rail::approx_equal(1.0, 1.0));
  EXPECT_TRUE(cda_rail::approx_equal(1.0, 1.0 + 5 * eps, 10.0));
  EXPECT_FALSE(cda_rail::approx_equal(1.0, 1.0 + 10 * eps, 10.0));
  EXPECT_THROW((void)cda_rail::approx_equal(1.0, 1.0, -1.0),
               cda_rail::exceptions::InvalidInputException);
}

TEST(GeneralHelper, RoundSmallNumbersToZeroInplace) {
  double val_pos = 1e-8;
  cda_rail::round_small_numbers_to_zero_inplace(val_pos, 1e-6);
  EXPECT_DOUBLE_EQ(val_pos, 0.0);

  double val_neg = -1e-8;
  cda_rail::round_small_numbers_to_zero_inplace(val_neg, 1e-6);
  EXPECT_DOUBLE_EQ(val_neg, 0.0);

  double val_equal_tol = 1e-6;
  cda_rail::round_small_numbers_to_zero_inplace(val_equal_tol, 1e-6);
  EXPECT_DOUBLE_EQ(val_equal_tol, 1e-6);

  double val_large = 2e-4;
  cda_rail::round_small_numbers_to_zero_inplace(val_large, 1e-6);
  EXPECT_DOUBLE_EQ(val_large, 2e-4);

  double any_val = 0.1;
  EXPECT_THROW(cda_rail::round_small_numbers_to_zero_inplace(any_val, -1e-9),
               cda_rail::exceptions::InvalidInputException);
}

TEST(GeneralHelper, RoundToGivenTolerance) {
  EXPECT_NEAR(cda_rail::round_to_given_tolerance(1.24, 0.5), 1.0,
              cda_rail::EPS);
  EXPECT_NEAR(cda_rail::round_to_given_tolerance(1.26, 0.5), 1.5,
              cda_rail::EPS);
  EXPECT_NEAR(cda_rail::round_to_given_tolerance(-1.26, 0.5), -1.5,
              cda_rail::EPS);
  EXPECT_NEAR(cda_rail::round_to_given_tolerance(0.44, 0.3), 0.3,
              10 * cda_rail::EPS);

  EXPECT_THROW((void)cda_rail::round_to_given_tolerance(1.0, 0.0),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW((void)cda_rail::round_to_given_tolerance(1.0, -0.1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(GeneralHelper, SubsetsOfSizeKThrowsForInvalidInput) {
  EXPECT_THROW((void)cda_rail::subsets_of_size_k_indices(3, 4),
               std::invalid_argument);
}

TEST(GeneralHelper, SubsetsOfSizeKZeroAndFullSetCases) {
  EXPECT_TRUE(cda_rail::subsets_of_size_k_indices(8, 0).empty());

  const auto full_subset = cda_rail::subsets_of_size_k_indices(4, 4);
  ASSERT_EQ(full_subset.size(), 1);
  EXPECT_EQ(full_subset.front(), (std::vector<size_t>{0, 1, 2, 3}));
}

TEST(GeneralHelper, SubsetsOfSizeKProperties) {
  constexpr size_t n       = 6;
  constexpr size_t k       = 3;
  const auto       subsets = cda_rail::subsets_of_size_k_indices(n, k);

  EXPECT_EQ(subsets.size(), choose(n, k));

  const bool all_valid =
      std::ranges::all_of(subsets, [n, k](const auto& subset) {
        return subset.size() == k && std::ranges::is_sorted(subset) &&
               std::ranges::adjacent_find(subset) == subset.end() &&
               std::ranges::all_of(subset, [n](size_t idx) { return idx < n; });
      });
  EXPECT_TRUE(all_valid);

  std::set<std::vector<size_t>> unique_subsets(subsets.begin(), subsets.end());
  EXPECT_EQ(unique_subsets.size(), subsets.size());
}

TEST(GeneralHelper, SubsetsOfSize2ThrowsForInvalidInput) {
  EXPECT_THROW((void)cda_rail::subsets_of_size_2_indices(0),
               std::invalid_argument);
  EXPECT_THROW((void)cda_rail::subsets_of_size_2_indices(1),
               std::invalid_argument);
}

TEST(GeneralHelper, SubsetsOfSize2ContainsExactlyAllPairs) {
  constexpr size_t n     = 5;
  const auto       pairs = cda_rail::subsets_of_size_2_indices(n);

  EXPECT_EQ(pairs.size(), choose(n, 2));

  std::set<std::pair<size_t, size_t>> expected;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      expected.emplace(i, j);
    }
  }
  std::set<std::pair<size_t, size_t>> actual(pairs.begin(), pairs.end());
  EXPECT_EQ(actual, expected);

  const bool all_pairs_ordered = std::ranges::all_of(
      pairs, [n](const auto& p) { return p.first < p.second && p.second < n; });
  EXPECT_TRUE(all_pairs_ordered);
}

TEST(GeneralHelper, SubsetsOfSizeK4) {
  constexpr size_t n       = 7;
  constexpr size_t k       = 4;
  const auto       subsets = cda_rail::subsets_of_size_k_indices(n, k);

  EXPECT_EQ(subsets.size(), choose(n, k));

  const bool all_valid = std::ranges::all_of(subsets, [n](const auto& subset) {
    return subset.size() == 4 && std::ranges::is_sorted(subset) &&
           std::ranges::adjacent_find(subset) == subset.end() &&
           std::ranges::all_of(subset, [n](size_t idx) { return idx < n; });
  });
  EXPECT_TRUE(all_valid);

  EXPECT_NE(std::ranges::find(subsets, std::vector<size_t>{0, 1, 2, 3}),
            subsets.end());
  EXPECT_NE(std::ranges::find(subsets, std::vector<size_t>{0, 2, 4, 6}),
            subsets.end());
  EXPECT_NE(std::ranges::find(subsets, std::vector<size_t>{3, 4, 5, 6}),
            subsets.end());
}

TEST(GeneralHelper, Subsets2) {
  const auto subsets_of_size_2 = cda_rail::subsets_of_size_2_indices(5);
  EXPECT_EQ(subsets_of_size_2.size(), 10);

  const std::vector<std::pair<size_t, size_t>> expected_pairs = {
      {0, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 2},
      {1, 3}, {1, 4}, {2, 3}, {2, 4}, {3, 4}};

  const bool all_expected_exist = std::ranges::all_of(
      expected_pairs, [&subsets_of_size_2](const auto& expected_pair) {
        return std::ranges::find(subsets_of_size_2, expected_pair) !=
               subsets_of_size_2.end();
      });
  EXPECT_TRUE(all_expected_exist);
}

TEST(GeneralHelper, Subsets3) {
  const auto subsets_of_size_3 = cda_rail::subsets_of_size_k_indices(6, 3);
  EXPECT_EQ(subsets_of_size_3.size(), 20);

  const std::vector<std::vector<size_t>> expected_subsets = {
      {0, 1, 2}, {0, 1, 3}, {0, 1, 4}, {0, 1, 5}, {0, 2, 3},
      {0, 2, 4}, {0, 2, 5}, {0, 3, 4}, {0, 3, 5}, {0, 4, 5},
      {1, 2, 3}, {1, 2, 4}, {1, 2, 5}, {1, 3, 4}, {1, 3, 5},
      {1, 4, 5}, {2, 3, 4}, {2, 3, 5}, {2, 4, 5}, {3, 4, 5}};

  const bool all_expected_exist = std::ranges::all_of(
      expected_subsets, [&subsets_of_size_3](const auto& expected_subset) {
        return std::ranges::find(subsets_of_size_3, expected_subset) !=
               subsets_of_size_3.end();
      });
  EXPECT_TRUE(all_expected_exist);
}

TEST(GeneralHelper, ConcatenateStringViews) {
  EXPECT_TRUE(cda_rail::concatenate_string_views({}).empty());
  EXPECT_EQ(cda_rail::concatenate_string_views({"abc"}), "abc");
  EXPECT_EQ(cda_rail::concatenate_string_views({"ab", "", "cd", "ef"}),
            "abcdef");

  const std::string left  = "left";
  const std::string right = "right";
  EXPECT_EQ(cda_rail::concatenate_string_views({left, "-", right}),
            "left-right");
}

TEST(GeneralHelper, InitializePlogCreatesAndSetsSeverity) {
  cda_rail::initialize_plog(true, true);
  ASSERT_NE(plog::get(), nullptr);
  EXPECT_EQ(plog::get()->getMaxSeverity(), plog::debug);

  // Without overwrite, severity should remain unchanged when currently > info.
  cda_rail::initialize_plog(false, false);
  EXPECT_EQ(plog::get()->getMaxSeverity(), plog::debug);

  cda_rail::initialize_plog(false, true);
  EXPECT_EQ(plog::get()->getMaxSeverity(), plog::info);
}

TEST(GeneralHelper, IsDirectoryAndCreate) {
  EXPECT_TRUE(cda_rail::is_directory_and_create(std::filesystem::path{}));

  const auto unique_suffix = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
  const auto test_root =
      std::filesystem::temp_directory_path() /
      std::filesystem::path("cda_rail_general_helper_test_" + unique_suffix);

  ASSERT_TRUE(std::filesystem::create_directories(test_root));
  EXPECT_TRUE(cda_rail::is_directory_and_create(test_root));

  const auto nested_dir = test_root / "a" / "b" / "c";
  EXPECT_TRUE(cda_rail::is_directory_and_create(nested_dir));
  EXPECT_TRUE(std::filesystem::exists(nested_dir));
  EXPECT_TRUE(std::filesystem::is_directory(nested_dir));

  const auto file_path = test_root / "plain_file.txt";
  {
    std::ofstream os(file_path.string());
    os << "content";
  }
  EXPECT_TRUE(std::filesystem::exists(file_path));
  EXPECT_FALSE(cda_rail::is_directory_and_create(file_path));

  std::error_code ec;
  std::filesystem::remove_all(test_root, ec);
}

TEST(GeneralHelper, BoolOptioan) {
  std::optional<bool> opt_bool;
  EXPECT_FALSE(opt_bool.has_value());

  cda_rail::to_bool_optional("test", opt_bool);
  EXPECT_FALSE(opt_bool.has_value());

  cda_rail::to_bool_optional("true", opt_bool);
  EXPECT_TRUE(opt_bool.has_value());
  EXPECT_TRUE(opt_bool.value());

  cda_rail::to_bool_optional("false", opt_bool);
  EXPECT_TRUE(opt_bool.has_value());
  EXPECT_FALSE(opt_bool.value());

  cda_rail::to_bool_optional("TRue", opt_bool);
  EXPECT_TRUE(opt_bool.has_value());
  EXPECT_TRUE(opt_bool.value());

  cda_rail::to_bool_optional("fALsE", opt_bool);
  EXPECT_TRUE(opt_bool.has_value());
  EXPECT_FALSE(opt_bool.value());
}

// NOLINTEND(clang-diagnostic-unused-result)
