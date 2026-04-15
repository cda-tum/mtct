#include "MultiArray.hpp"

#include <cstddef>
#include <sstream>
#include <stdexcept>

namespace cda_rail {

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
  m_shape = dims;

  size_t cap = 1;
  for (const auto& d : dims) {
    cap *= d;
  }
  m_data.delete_and_resize(cap);
}

// Explicit instantiation – generates compiled symbols for size_t.
// Add further instantiations here for other element types as needed.
template class MultiArray<size_t>;

} // namespace cda_rail
