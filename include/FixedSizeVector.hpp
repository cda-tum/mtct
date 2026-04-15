#pragma once
#include <cstddef>

namespace cda_rail {

using std::size_t;

template <typename T> class FixedSizeVector {
private:
  std::unique_ptr<T[]> m_data{std::make_unique<T[]>(0)};
  size_t               m_len{0};

public:
  FixedSizeVector() = default;
  explicit FixedSizeVector(size_t const len)
      : m_data(std::make_unique<T[]>(len)), m_len(len) {}

  // Rule of 5
  FixedSizeVector(const FixedSizeVector& other)
      : m_data(std::make_unique<T[]>(other.m_len)), m_len(other.m_len) {
    std::copy(other.begin(), other.end(), begin());
  }
  FixedSizeVector& operator=(const FixedSizeVector& other) {
    if (this == &other) {
      return *this;
    }
    FixedSizeVector tmp(other);
    std::swap(m_data, tmp.m_data);
    std::swap(m_len, tmp.m_len);
    return *this;
  }
  FixedSizeVector(FixedSizeVector&&) noexcept            = default;
  FixedSizeVector& operator=(FixedSizeVector&&) noexcept = default;
  ~FixedSizeVector()                                     = default;

  void delete_and_resize(size_t const len) {
    m_data = std::make_unique<T[]>(len);
    m_len  = len;
  }

  [[nodiscard]] T&       operator[](size_t const i) { return m_data[i]; }
  [[nodiscard]] const T& operator[](size_t const i) const { return m_data[i]; }

  // Bounds-safe element access.
  [[nodiscard]] T& at(size_t const i) {
    if (i >= m_len) {
      throw std::out_of_range("FixedSizeVector index out of range.");
    }
    return m_data[i];
  }
  [[nodiscard]] const T& at(size_t const i) const {
    if (i >= m_len) {
      throw std::out_of_range("FixedSizeVector index out of range.");
    }
    return m_data[i];
  }

  [[nodiscard]] T*       begin() { return m_data.get(); }
  [[nodiscard]] T*       end() { return m_data.get() + m_len; }
  [[nodiscard]] const T* begin() const { return m_data.get(); }
  [[nodiscard]] const T* end() const { return m_data.get() + m_len; }
  [[nodiscard]] const T* cbegin() const { return begin(); }
  [[nodiscard]] const T* cend() const { return end(); }

  [[nodiscard]] size_t size() const { return m_len; }
};

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

template <typename T>
bool operator==(const std::vector<T>& lhs, const FixedSizeVector<T>& rhs) {
  return rhs == lhs;
}
} // namespace cda_rail
