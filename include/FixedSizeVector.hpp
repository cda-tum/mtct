#pragma once
#include <algorithm>
#include <cstddef>
#include <initializer_list>
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
 *           array is zero-initialized on construction.
 *
 * @invariant All indices in `[0, size())` refer to valid, initialized elements.
 */
// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays)
template <typename T> class FixedSizeVector {
private:
  // NOLINTBEGIN(*-avoid-c-arrays)
  std::unique_ptr<T[]> m_data{std::make_unique<T[]>(0)};
  // NOLINTEND(*-avoid-c-arrays)
  size_t m_len{0};

public:
  /*
   * CONSTRUCTORS / RULE OF 5
   */

  /**
   * @brief Constructs an empty vector with zero elements.
   */
  FixedSizeVector() = default;

  /**
   * @brief Constructs a vector with @p len default-initialized elements.
   *
   * @param len Number of elements to allocate.
   */
  explicit FixedSizeVector(size_t const len)
      // NOLINTNEXTLINE(*-avoid-c-arrays)
      : m_data(std::make_unique<T[]>(len)), m_len(len) {}

  /**
   * @brief Constructs a fixed size vector with given values.
   *
   * @param init Values to store in a fixed vector
   */
  FixedSizeVector(std::initializer_list<T> init)
      // NOLINTNEXTLINE(*-avoid-c-arrays)
      : m_data(std::make_unique<T[]>(init.size())), m_len(init.size()) {
    std::copy(init.begin(), init.end(), begin());
  }

  /**
   * @brief Creates a deep copy of another vector's elements.
   *
   * @param other Source vector to copy from.
   */
  FixedSizeVector(const FixedSizeVector& other)
      // NOLINTNEXTLINE(*-avoid-c-arrays)
      : m_data(std::make_unique<T[]>(other.m_len)), m_len(other.m_len) {
    std::copy(other.begin(), other.end(), begin());
  }

  /**
   * @brief Assigns a copy of another vector to this vector.
   *
   * @param other Source vector to copy from.
   * @return Reference to this vector.
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
   * @brief Transfers ownership of the managed array from another vector.
   *
   * After the move, @p other is in a valid but unspecified state.
   */
  FixedSizeVector(FixedSizeVector&& other) noexcept
      : m_data(std::move(other.m_data)), m_len(other.m_len) {
    other.m_len = 0;
  }

  /**
   * @brief Move-assigns from another FixedSizeVector.
   *
   * After the move, the source object is in a valid but unspecified state.
   * @return Reference to this object.
   */
  FixedSizeVector& operator=(FixedSizeVector&& other) noexcept {
    if (this != &other) {
      m_data      = std::move(other.m_data);
      m_len       = other.m_len;
      other.m_len = 0;
    }
    return *this;
  }

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
   * default-initialized.
   *
   * @param len New number of elements.
   */
  void delete_and_resize(size_t const len) {
    // NOLINTNEXTLINE(*-avoid-c-arrays)
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
   * @pre `i < size()`.  Behavior is undefined if the precondition is violated.
   */
  [[nodiscard]] T& operator[](size_t const i) { return m_data[i]; }

  /**
   * @brief Accesses an element at a specific index without bounds checking.
   *
   * @return Const reference to the element at position `i`.
   * @pre `i < size()`. Behavior is undefined if the precondition is violated.
   */
  [[nodiscard]] const T& operator[](size_t const i) const { return m_data[i]; }

  /**
   * @brief Accesses an element at a given index with bounds checking.
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
   * @brief Accesses the first element of the underlying array.
   *
   * @return Pointer to the first element.
   */
  [[nodiscard]] T* begin() { return m_data.get(); }

  /**
   * @brief Provides a pointer to the position past the last element.
   *
   * @return Pointer to one past the last element.
   */
  [[nodiscard]] T* end() { return m_data.get() + m_len; }

  /**
   * @brief Obtains a const pointer to the first element.
   *
   * @return Const pointer to the first element, or a pointer equal to `cend()`
   * if empty.
   */
  [[nodiscard]] const T* begin() const { return m_data.get(); }

  /**
   * @brief Obtains a const pointer to one past the last element.
   *
   * @return A const pointer one past the last element.
   */
  [[nodiscard]] const T* end() const { return m_data.get() + m_len; }

  /**
   * @brief Obtains a const pointer to the first element.
   *
   * @return Const pointer to the first element.
   */
  [[nodiscard]] const T* cbegin() const { return begin(); }

  /**
   * @brief Provides a const pointer to one past the last element.
   *
   * @return const T* A const pointer to one past the last element.
   */
  [[nodiscard]] const T* cend() const { return end(); }

  /*
   * CAPACITY
   */

  /**
   * @brief Queries the number of elements.
   *
   * @return The number of elements in the vector (fixed after construction
   * until `delete_and_resize` is called).
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
/**
 * @brief Compares a FixedSizeVector with a std::vector for equality.
 *
 * @return `true` if the containers are equal, `false` otherwise.
 */
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
/**
 * @brief Determines whether a std::vector and FixedSizeVector are equal.
 *
 * @tparam T Element type.
 * @param lhs The std::vector to compare.
 * @param rhs The FixedSizeVector to compare.
 * @return `true` if the vector and FixedSizeVector have equal size and
 * identical elements, `false` otherwise.
 */
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
/**
 * @brief Exchanges the contents of two FixedSizeVector objects.
 */
void swap(FixedSizeVector<T>& lhs, FixedSizeVector<T>& rhs) noexcept {
  lhs.swap(rhs);
}

// NOLINTEND(cppcoreguidelines-avoid-c-arrays)
} // namespace cda_rail
