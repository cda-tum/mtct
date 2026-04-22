#pragma once
#include "FixedSizeVector.hpp"

#include <cstddef>
#include <sstream>
#include <stdexcept>

namespace cda_rail {

/**
 * @brief A dynamically-shaped N-dimensional array stored in row-major order.
 *
 * The shape (number of dimensions and the extent of each dimension) is fixed at
 * construction time.  Element storage is a flat `FixedSizeVector<T>` whose
 * length equals the product of all dimension extents.  Multi-dimensional
 * indices are linearised using a row-major (last-index-fastest) scheme.
 *
 * A zero-dimensional `MultiArray` behaves like a scalar: it contains exactly
 * one element that is accessed via `operator()()` or `at()`.
 *
 * @tparam T Element type.  Must be default-constructible.
 *
 * @invariant `m_data.size()` equals the product of all extents in `m_shape`
 *            (or 1 when `m_shape` is empty).
 */
template <typename T> class MultiArray {
private:
  FixedSizeVector<size_t> m_shape; ///< Extent of each dimension.
  FixedSizeVector<T>      m_data;  ///< Flat element storage in row-major order.

  /**
   * @brief Packs variadic arguments into a `FixedSizeVector<size_t>`.
   *
   * @tparam Args Integral types that are cast to `size_t`.
   * @param args Dimension indices or extents to pack.
   * @return `FixedSizeVector` containing each argument cast to `size_t`,
   *         in the order they were supplied.
   */
  template <typename... Args>
  [[nodiscard]] static FixedSizeVector<size_t> make_index_vector(Args... args) {
    return FixedSizeVector<size_t>{static_cast<size_t>(args)...};
  }

  /**
   * @brief Validates that @p args matches the array's shape.
   *
   * @param args Index vector to validate.
   * @throws std::invalid_argument If `args.size() != dimensions()`.
   * @throws std::out_of_range If any `args[i] >= m_shape[i]`.
   */
  void check_args(const FixedSizeVector<size_t>& args) const;

  /**
   * @brief Converts a multi-dimensional index vector to a flat storage index.
   *
   * Calls `check_args` internally before computing the index.
   *
   * @param args Validated multi-dimensional index vector.
   * @return Flat index into `m_data` corresponding to @p args.
   * @throws std::invalid_argument If `args.size() != dimensions()`.
   * @throws std::out_of_range If any `args[i] >= m_shape[i]`.
   */
  [[nodiscard]] size_t flat_index(const FixedSizeVector<size_t>& args) const;

  /**
   * @brief Initialises shape and allocates element storage.
   *
   * Sets `m_shape` to @p dims and allocates a flat `m_data` buffer whose size
   * is the product of all extents (minimum 1).
   *
   * @param dims Extents of each dimension.
   */
  void init(const FixedSizeVector<size_t>& dims);

public:
  /*
   * CONSTRUCTORS
   */

  /**
   * @brief Constructs a `MultiArray` with the given dimension extents.
   *
   * Pass one extent per dimension (e.g. `MultiArray<int>(3, 4)` creates a 3×4
   * matrix).  Passing no arguments creates a zero-dimensional scalar array
   * containing exactly one element.
   *
   * @tparam Args Integral types convertible to `size_t`.
   * @param args Extents of each dimension, in order.
   */
  template <typename... Args> explicit MultiArray(Args... args) {
    init(make_index_vector(args...));
  }

  /*
   * ELEMENT ACCESS
   */

  /**
   * @brief Unchecked multi-dimensional element access (non-const).
   *
   * The number of arguments must equal `dimensions()` and each index must be
   * within bounds; these conditions are enforced via `flat_index`/`check_args`.
   *
   * @tparam Args Integral types convertible to `size_t`.
   * @param args One index per dimension.
   * @return Reference to the requested element.
   * @throws std::invalid_argument If the number of arguments differs from
   *         `dimensions()`.
   * @throws std::out_of_range If any index exceeds its dimension's extent.
   */
  template <typename... Args> T& operator()(Args... args) {
    return m_data[flat_index(make_index_vector(args...))];
  }

  /**
   * @brief Bounds-checked multi-dimensional element access (non-const).
   *
   * @tparam Args Integral types convertible to `size_t`.
   * @param args One index per dimension.
   * @return Reference to the requested element.
   * @throws std::invalid_argument If the number of arguments differs from
   *         `dimensions()`.
   * @throws std::out_of_range If any index exceeds its dimension's extent.
   */
  template <typename... Args> [[nodiscard]] T& at(Args... args) {
    return m_data.at(flat_index(make_index_vector(args...)));
  }

  /**
   * @brief Bounds-checked multi-dimensional element access (const).
   *
   * @tparam Args Integral types convertible to `size_t`.
   * @param args One index per dimension.
   * @return Const reference to the requested element.
   * @throws std::invalid_argument If the number of arguments differs from
   *         `dimensions()`.
   * @throws std::out_of_range If any index exceeds its dimension's extent.
   */
  template <typename... Args> [[nodiscard]] const T& at(Args... args) const {
    return m_data.at(flat_index(make_index_vector(args...)));
  }

  /*
   * SHAPE / SIZE QUERIES
   */

  /**
   * @brief Returns the shape of the array.
   *
   * @return Const reference to the `FixedSizeVector` holding each dimension's
   *         extent.  The vector's size equals `dimensions()`.
   */
  [[nodiscard]] const FixedSizeVector<size_t>& get_shape() const {
    return m_shape;
  }

  /**
   * @brief Returns the total number of elements.
   *
   * @return Product of all dimension extents (1 for a zero-dimensional array).
   */
  [[nodiscard]] size_t size() const { return m_data.size(); }

  /**
   * @brief Returns the number of dimensions.
   *
   * @return Number of dimensions (0 for a scalar, 1 for a vector, 2 for a
   *         matrix, etc.).
   */
  [[nodiscard]] size_t dimensions() const { return m_shape.size(); }
};

template <typename T>
void MultiArray<T>::check_args(const FixedSizeVector<size_t>& args) const {
  if (m_shape.size() != args.size()) {
    throw std::invalid_argument(
        "Number of dimensions and number of arguments do not coincide.");
  }
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] >= m_shape[i]) {
      std::stringstream ss;
      ss << "Index " << args[i] << " is too large for dimension " << i;
      throw std::out_of_range(ss.str());
    }
  }
}

template <typename T>
size_t MultiArray<T>::flat_index(const FixedSizeVector<size_t>& args) const {
  check_args(args);
  size_t index      = 0;
  size_t multiplier = 1;
  for (size_t i = 0; i < args.size(); ++i) {
    index += args[i] * multiplier;
    multiplier *= m_shape[i];
  }
  return index;
}

template <typename T>
void MultiArray<T>::init(const FixedSizeVector<size_t>& dims) {
  m_shape    = dims;
  size_t cap = 1;
  for (const auto& d : dims) {
    cap *= d;
  }
  m_data.delete_and_resize(cap);
}

} // namespace cda_rail
