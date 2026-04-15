#pragma once
#include "Definitions.hpp"
#include "FixedSizeVector.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cda_rail {
template <typename T> class MultiArray {
private:
  FixedSizeVector<size_t> m_shape;
  FixedSizeVector<T>      m_data;

  template <typename... Args>
  [[nodiscard]] static FixedSizeVector<size_t> make_index_vector(Args... args) {
    FixedSizeVector<size_t> idx(sizeof...(args));
    size_t                  i = 0;
    ((idx[i++] = static_cast<size_t>(args)), ...);
    return idx;
  }

  void                 check_args(const FixedSizeVector<size_t>& args) const;
  [[nodiscard]] size_t flat_index(const FixedSizeVector<size_t>& args) const;
  void                 init(const FixedSizeVector<size_t>& dims);

public:
  template <typename... Args> explicit MultiArray(Args... args) {
    init(make_index_vector(args...));
  }

  template <typename... Args> T& operator()(Args... args) {
    return m_data[flat_index(make_index_vector(args...))];
  }

  template <typename... Args> [[nodiscard]] T& at(Args... args) {
    return m_data.at(flat_index(make_index_vector(args...)));
  }

  template <typename... Args> [[nodiscard]] const T& at(Args... args) const {
    return m_data.at(flat_index(make_index_vector(args...)));
  }

  [[nodiscard]] const FixedSizeVector<size_t>& get_shape() const {
    return m_shape;
  }
  [[nodiscard]] size_t size() const { return m_data.size(); }
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
