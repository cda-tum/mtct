#pragma once
#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <vector>

namespace cda_rail {

using std::size_t;

/**
 * @brief A heap-allocated, fixed-size array whose capacity cannot change after
 *        construction.
 *
 * Provides a lightweight alternative to `std::vector<T>` for situations where
 * the number of elements is determined at run time but remains constant for the
 * lifetime of the object.  The storage is managed through a
 * `std::unique_ptr<T[]>`, giving deterministic ownership without the capacity
 * overhead of `std::vector`.
 *
 * @tparam T Element type.  Must be default-constructible because the underlying
 *           array is zero-initialised on construction.
 *
 * @invariant All indices in `[0, size())` refer to valid, initialised elements.
 */
template <typename T> class FixedSizeVector {
private:
  std::unique_ptr<T[]> m_data{std::make_unique<T[]>(0)};
  size_t               m_len{0};

public:
  /*
   * CONSTRUCTORS / RULE OF 5
   */

  /**
   * @brief Constructs an empty vector with zero elements.
   */
  FixedSizeVector() = default;

  /**
   * @brief Constructs a vector with @p len default-initialised elements.
   *
   * @param len Number of elements to allocate.
   */
  explicit FixedSizeVector(size_t const len)
      : m_data(std::make_unique<T[]>(len)), m_len(len) {}

  /**
   * @brief Copy constructor.  Performs a deep copy of all elements.
   *
   * @param other Source vector to copy from.
   */
  FixedSizeVector(const FixedSizeVector& other)
      : m_data(std::make_unique<T[]>(other.m_len)), m_len(other.m_len) {
    std::copy(other.begin(), other.end(), begin());
  }

  /**
   * @brief Copy-assignment operator.  Performs a deep copy via copy-and-swap.
   *
   * @param other Source vector to copy from.
   * @return Reference to this object after assignment.
   */
  FixedSizeVector& operator=(const FixedSizeVector& other) {
    if (this == &other) {
      return *this;
    }
    FixedSizeVector tmp(other);
    swap(tmp);
    return *this;
  }

  /**
   * @brief Move constructor.  Transfers ownership from @p other in O(1).
   *
   * After the move, @p other is in a valid but unspecified state.
   */
  FixedSizeVector(FixedSizeVector&&) noexcept = default;

  /**
   * @brief Move-assignment operator.  Transfers ownership from @p other in
   * O(1).
   *
   * After the move, @p other is in a valid but unspecified state.
   * @return Reference to this object after assignment.
   */
  FixedSizeVector& operator=(FixedSizeVector&&) noexcept = default;

  /**
   * @brief Destructor.  Releases the managed array.
   */
  ~FixedSizeVector() = default;

  /*
   * MODIFIERS
   */

  /**
   * @brief Swaps the contents of this vector with @p other.
   *
   * @param other Vector to swap with.
   */
  void swap(FixedSizeVector& other) noexcept {
    std::swap(m_data, other.m_data);
    std::swap(m_len, other.m_len);
  }

  /**
   * @brief Replaces the storage with a new allocation of @p len elements.
   *
   * All existing elements are discarded and the new elements are
   * default-initialised.
   *
   * @param len New number of elements.
   */
  void delete_and_resize(size_t const len) {
    m_data = std::make_unique<T[]>(len);
    m_len  = len;
  }

  /*
   * ELEMENT ACCESS
   */

  /**
   * @brief Unchecked element access (non-const).
   *
   * @param i Zero-based index of the element.
   * @return Reference to the element at position @p i.
   * @pre `i < size()`.  Behaviour is undefined if the precondition is violated.
   */
  [[nodiscard]] T& operator[](size_t const i) { return m_data[i]; }

  /**
   * @brief Unchecked element access (const).
   *
   * @param i Zero-based index of the element.
   * @return Const reference to the element at position @p i.
   * @pre `i < size()`.  Behaviour is undefined if the precondition is violated.
   */
  [[nodiscard]] const T& operator[](size_t const i) const { return m_data[i]; }

  /**
   * @brief Bounds-checked element access (non-const).
   *
   * @param i Zero-based index of the element.
   * @return Reference to the element at position @p i.
   * @throws std::out_of_range If `i >= size()`.
   */
  [[nodiscard]] T& at(size_t const i) {
    if (i >= m_len) {
      throw std::out_of_range("FixedSizeVector index out of range.");
    }
    return m_data[i];
  }

  /**
   * @brief Bounds-checked element access (const).
   *
   * @param i Zero-based index of the element.
   * @return Const reference to the element at position @p i.
   * @throws std::out_of_range If `i >= size()`.
   */
  [[nodiscard]] const T& at(size_t const i) const {
    if (i >= m_len) {
      throw std::out_of_range("FixedSizeVector index out of range.");
    }
    return m_data[i];
  }

  /*
   * ITERATORS
   */

  /**
   * @brief Returns a pointer to the first element (non-const iterator).
   *
   * @return Pointer to the first element, or `end()` if the vector is empty.
   */
  [[nodiscard]] T* begin() { return m_data.get(); }

  /**
   * @brief Returns a pointer one past the last element (non-const sentinel).
   *
   * @return Pointer one past the last element.
   */
  [[nodiscard]] T* end() { return m_data.get() + m_len; }

  /**
   * @brief Returns a const pointer to the first element.
   *
   * @return Const pointer to the first element, or `cend()` if empty.
   */
  [[nodiscard]] const T* begin() const { return m_data.get(); }

  /**
   * @brief Returns a const pointer one past the last element.
   *
   * @return Const pointer one past the last element.
   */
  [[nodiscard]] const T* end() const { return m_data.get() + m_len; }

  /**
   * @brief Returns a const pointer to the first element.
   *
   * @return Const pointer to the first element, or `cend()` if empty.
   */
  [[nodiscard]] const T* cbegin() const { return begin(); }

  /**
   * @brief Returns a const pointer one past the last element.
   *
   * @return Const pointer one past the last element.
   */
  [[nodiscard]] const T* cend() const { return end(); }

  /*
   * CAPACITY
   */

  /**
   * @brief Returns the number of elements.
   *
   * @return Number of elements in the vector (fixed after construction until
   *         `delete_and_resize` is called).
   */
  [[nodiscard]] size_t size() const { return m_len; }
};

/**
 * @brief Equality comparison between a `FixedSizeVector` and a `std::vector`.
 *
 * @tparam T Element type.
 * @param lhs Left-hand `FixedSizeVector`.
 * @param rhs Right-hand `std::vector`.
 * @return `true` if both containers have the same size and identical elements
 *         at every index, otherwise `false`.
 */
template <typename T>
bool operator==(const FixedSizeVector<T>& lhs, const std::vector<T>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i] != rhs[i]) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Equality comparison between a `std::vector` and a `FixedSizeVector`.
 *
 * @tparam T Element type.
 * @param lhs Left-hand `std::vector`.
 * @param rhs Right-hand `FixedSizeVector`.
 * @return `true` if both containers have the same size and identical elements
 *         at every index, otherwise `false`.
 */
template <typename T>
bool operator==(const std::vector<T>& lhs, const FixedSizeVector<T>& rhs) {
  return rhs == lhs;
}

/**
 * @brief ADL-reachable swap for `FixedSizeVector`.
 *
 * @tparam T Element type.
 * @param lhs First vector.
 * @param rhs Second vector.
 */
template <typename T>
void swap(FixedSizeVector<T>& lhs, FixedSizeVector<T>& rhs) noexcept {
  lhs.swap(rhs);
}

} // namespace cda_rail
