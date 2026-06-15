#include "MultiArray.hpp"

#include "gtest/gtest.h"
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using std::size_t;

TEST(FixedSizeVector, DefaultConstructionAndResize) {
  cda_rail::FixedSizeVector<size_t> vec;
  EXPECT_EQ(vec.size(), 0);
  EXPECT_THROW((void)vec.at(0), std::out_of_range);

  vec.delete_and_resize(4);
  EXPECT_EQ(vec.size(), 4);

  for (size_t i = 0; i < vec.size(); ++i) {
    vec.at(i) = 10 + i;
  }

  EXPECT_EQ(vec.at(0), 10);
  EXPECT_EQ(vec.at(1), 11);
  EXPECT_EQ(vec.at(2), 12);
  EXPECT_EQ(vec.at(3), 13);
}

TEST(FixedSizeVector, ConstructorWithInitializerList) {
  cda_rail::FixedSizeVector<size_t> vec{1, 2, 3, 4};
  EXPECT_EQ(vec.size(), 4);
  EXPECT_EQ(vec.at(0), 1);
  EXPECT_EQ(vec.at(1), 2);
  EXPECT_EQ(vec.at(2), 3);
  EXPECT_EQ(vec.at(3), 4);
}

TEST(FixedSizeVector, AtChecksBoundsForConstAndNonConst) {
  cda_rail::FixedSizeVector<size_t> vec(3);
  vec.at(0) = 7;
  vec.at(1) = 8;
  vec.at(2) = 9;

  EXPECT_EQ(vec.at(0), 7);
  EXPECT_EQ(vec.at(2), 9);
  EXPECT_THROW((void)vec.at(3), std::out_of_range);

  const auto& cvec = vec;
  EXPECT_EQ(cvec.at(1), 8);
  EXPECT_THROW((void)cvec.at(3), std::out_of_range);
}

TEST(FixedSizeVector, CopyOperationsCreateDeepCopies) {
  cda_rail::FixedSizeVector<size_t> original(3);
  original.at(0) = 1;
  original.at(1) = 2;
  original.at(2) = 3;

  cda_rail::FixedSizeVector<size_t> copied(original);
  copied.at(1) = 99;

  EXPECT_EQ(original.at(1), 2);
  EXPECT_EQ(copied.at(1), 99);

  cda_rail::FixedSizeVector<size_t> assigned(1);
  assigned.at(0) = 42;
  assigned       = original;
  assigned.at(2) = 88;

  EXPECT_EQ(original.at(2), 3);
  EXPECT_EQ(assigned.at(2), 88);
  EXPECT_EQ(assigned.size(), 3);
}

TEST(FixedSizeVector, AssigningToSelfWorks) {
  cda_rail::FixedSizeVector<size_t> original(3);
  original.at(0) = 1;
  original.at(1) = 2;
  original.at(2) = 3;

  original = original;

  EXPECT_EQ(original.at(0), 1);
  EXPECT_EQ(original.at(1), 2);
  EXPECT_EQ(original.at(2), 3);
}

TEST(FixedSizeVector, SupportsRangeForIteration) {
  cda_rail::FixedSizeVector<size_t> vec(5);
  for (size_t i = 0; i < vec.size(); ++i) {
    vec.at(i) = i + 1;
  }

  size_t sum = 0;
  for (auto element : vec) {
    sum += element;
  }
  EXPECT_EQ(sum, 15);

  const auto& cvec    = vec;
  size_t      product = 1;
  for (const auto element : cvec) {
    product *= element;
  }
  EXPECT_EQ(product, 120);
}

TEST(FixedSizeVector, SupportsStringDataType) {
  cda_rail::FixedSizeVector<std::string> words(3);
  words.at(0) = "rail";
  words.at(1) = "signal";
  words.at(2) = "block";

  EXPECT_EQ(words.size(), 3);
  EXPECT_EQ(words.at(0), "rail");
  EXPECT_EQ(words.at(1), "signal");
  EXPECT_EQ(words.at(2), "block");

  std::string joined;
  for (const auto& word : words) {
    joined += word;
  }
  EXPECT_EQ(joined, "railsignalblock");
}

TEST(FixedSizeVector, MoveConstructionAndMoveAssignmentKeepElements) {
  cda_rail::FixedSizeVector<std::string> source(2);
  source.at(0) = "A";
  source.at(1) = "B";

  cda_rail::FixedSizeVector<std::string> moved_constructed(std::move(source));
  EXPECT_EQ(moved_constructed.size(), 2);
  EXPECT_EQ(moved_constructed.at(0), "A");
  EXPECT_EQ(moved_constructed.at(1), "B");

  cda_rail::FixedSizeVector<std::string> other(1);
  other.at(0) = "X";
  other       = std::move(moved_constructed);

  EXPECT_EQ(other.size(), 2);
  EXPECT_EQ(other.at(0), "A");
  EXPECT_EQ(other.at(1), "B");
}

TEST(MultiArray, BasicFunctionality) {
  cda_rail::MultiArray<size_t> a1(1, 2, 3);

  // Set elements
  for (size_t i = 0; i < 1; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t k = 0; k < 3; ++k) {
        a1(i, j, k) = (6 * i) + (3 * j) + k;
      }
    }
  }

  // Check elements
  for (size_t i = 0; i < 1; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t k = 0; k < 3; ++k) {
        EXPECT_EQ(a1(i, j, k), (6 * i) + ((3 * j) + k));
      }
    }
  }

  EXPECT_EQ(a1.size(), 6);
  EXPECT_EQ(a1.dimensions(), 3);
  auto const& shape = a1.get_shape();
  EXPECT_EQ(shape.size(), 3);
  EXPECT_EQ(shape.at(0), 1);
  EXPECT_EQ(shape.at(1), 2);
  EXPECT_EQ(shape.at(2), 3);

  // Calling with wrong number of arguments should throw std::invalid_argument
  EXPECT_THROW(a1(0), std::invalid_argument);
  EXPECT_THROW(a1(0, 0), std::invalid_argument);
  EXPECT_THROW(a1(0, 0, 0, 0), std::invalid_argument);

  // Calling with index too large should throw std::out_of_range
  EXPECT_THROW(a1(1, 0, 0), std::out_of_range);
  EXPECT_THROW(a1(0, 2, 0), std::out_of_range);
  EXPECT_THROW(a1(0, 0, 3), std::out_of_range);
}

TEST(MultiArray, AtProvidesBoundsSafeAccess) {
  cda_rail::MultiArray<size_t> matrix(2, 3);

  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      matrix.at(i, j) = (10 * i) + j;
    }
  }

  EXPECT_EQ(matrix.at(0, 0), 0);
  EXPECT_EQ(matrix.at(1, 2), 12);
  EXPECT_EQ(matrix(1, 1), 11);

  const auto& cmatrix = matrix;
  EXPECT_EQ(cmatrix.at(0, 2), 2);

  EXPECT_THROW((void)matrix.at(2, 0), std::out_of_range);
  EXPECT_THROW((void)matrix.at(0, 3), std::out_of_range);
  EXPECT_THROW((void)matrix.at(0), std::invalid_argument);
  EXPECT_THROW((void)matrix.at(0, 0, 0), std::invalid_argument);
}

TEST(MultiArray, ZeroDimensionalArrayBehavesLikeScalar) {
  cda_rail::MultiArray<size_t> scalar;

  EXPECT_EQ(scalar.size(), 1);
  EXPECT_EQ(scalar.dimensions(), 0);
  EXPECT_EQ(scalar.get_shape().size(), 0);

  scalar() = 123;
  EXPECT_EQ(scalar(), 123);
  EXPECT_EQ(scalar.at(), 123);

  const auto& cscalar = scalar;
  EXPECT_EQ(cscalar.at(), 123);

  EXPECT_THROW(scalar(0), std::invalid_argument);
  EXPECT_THROW((void)scalar.at(0), std::invalid_argument);
}

TEST(MultiArray, OneDimensionalArrayIndexingAndShape) {
  cda_rail::MultiArray<size_t> arr(4);

  for (size_t i = 0; i < 4; ++i) {
    arr(i) = i * i;
  }

  EXPECT_EQ(arr.size(), 4);
  EXPECT_EQ(arr.dimensions(), 1);
  EXPECT_EQ(arr.get_shape().size(), 1);
  EXPECT_EQ(arr.get_shape().at(0), 4);

  EXPECT_EQ(arr(0), 0);
  EXPECT_EQ(arr(1), 1);
  EXPECT_EQ(arr(2), 4);
  EXPECT_EQ(arr(3), 9);

  EXPECT_THROW(arr(4), std::out_of_range);
  EXPECT_THROW((void)arr.at(4), std::out_of_range);
}

TEST(MultiArray, FourDimensionalIndexingWorksConsistently) {
  cda_rail::MultiArray<size_t> hyper(2, 2, 2, 2);

  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t k = 0; k < 2; ++k) {
        for (size_t l = 0; l < 2; ++l) {
          hyper(i, j, k, l) = (((((i * 2) + j) * 2) + k) * 2) + l;
        }
      }
    }
  }

  EXPECT_EQ(hyper.size(), 16);
  EXPECT_EQ(hyper.dimensions(), 4);
  EXPECT_EQ(hyper(0, 0, 0, 0), 0);
  EXPECT_EQ(hyper(0, 1, 1, 1), 7);
  EXPECT_EQ(hyper(1, 0, 1, 0), 10);
  EXPECT_EQ(hyper(1, 1, 1, 1), 15);
}

TEST(MultiArray, ShapeIsIterableAndMatchesConstructorArguments) {
  cda_rail::MultiArray<size_t> const array(3, 1, 4);

  std::vector<size_t> shape_values;
  for (const auto dim : array.get_shape()) {
    shape_values.push_back(dim);
  }

  EXPECT_EQ(shape_values.size(), 3);
  EXPECT_EQ(shape_values.at(0), 3);
  EXPECT_EQ(shape_values.at(1), 1);
  EXPECT_EQ(shape_values.at(2), 4);
}

TEST(MultiArray, SupportsStringDataType) {
  cda_rail::MultiArray<std::string> grid(2, 2);

  grid(0, 0)    = "A";
  grid(0, 1)    = "B";
  grid.at(1, 0) = "C";
  grid.at(1, 1) = "D";

  EXPECT_EQ(grid.size(), 4);
  EXPECT_EQ(grid.dimensions(), 2);
  EXPECT_EQ(grid(0, 0), "A");
  EXPECT_EQ(grid(0, 1), "B");
  EXPECT_EQ(grid.at(1, 0), "C");
  EXPECT_EQ(grid.at(1, 1), "D");

  const auto& cgrid = grid;
  EXPECT_EQ(cgrid.at(1, 1), "D");
}

TEST(MultiArray, StringTypeThrowsOnInvalidIndices) {
  cda_rail::MultiArray<std::string> values(2, 1);

  values(0, 0) = "left";
  values(1, 0) = "right";

  EXPECT_THROW((void)values.at(2, 0), std::out_of_range);
  EXPECT_THROW((void)values.at(0, 1), std::out_of_range);
  EXPECT_THROW((void)values.at(0), std::invalid_argument);
  EXPECT_THROW((void)values.at(0, 0, 0), std::invalid_argument);
}
