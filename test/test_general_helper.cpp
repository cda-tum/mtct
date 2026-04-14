#include "GeneralHelper.hpp"

#include "gtest/gtest.h"

// NOLINTBEGIN(clang-diagnostic-unused-result)

TEST(Functionality, Subsets2) {
  auto subsets_of_size_2 = cda_rail::subsets_of_size_2_indices(5);
  // Expect 5 choose 2 number of elements
  EXPECT_EQ(subsets_of_size_2.size(), 10);
  // Expect all subsets to exist
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(0, 1)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(0, 2)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(0, 3)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(0, 4)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(1, 2)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(1, 3)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(1, 4)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(2, 3)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(2, 4)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(3, 4)) !=
              subsets_of_size_2.end());
}

TEST(Functionality, Subsets3) {
  auto subsets_of_size_3 = cda_rail::subsets_of_size_k_indices(6, 3);
  // Expect 6 choose 2 number of elements
  EXPECT_EQ(subsets_of_size_3.size(), 20);
  // Expect to find all subsets of size 3
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 1, 2})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 1, 3})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 1, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 1, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 2, 3})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 2, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 2, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 3, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 3, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 4, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 2, 3})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 2, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 2, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 3, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 3, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 4, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({2, 3, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({2, 3, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({2, 4, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({3, 4, 5})) !=
              subsets_of_size_3.end());
}

// NOLINTEND(clang-diagnostic-unused-result)
