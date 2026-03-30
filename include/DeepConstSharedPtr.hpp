#pragma once

#include <memory>

namespace cda_rail {
template <typename T> class DeepConstSharedPtr {
private:
  std::shared_ptr<T> ptr;

public:
  explicit DeepConstSharedPtr(std::shared_ptr<T> p) : ptr(std::move(p)) {
    // throw error if nullptr
    if (ptr == nullptr) {
      throw std::invalid_argument("DeepConstSharedPtr cannot be nullptr");
    }
  }

  // Non-const access returns standard pointers/references
  T* operator->() { return ptr.get(); }
  T& operator*() { return *ptr; }

  // Const access forces const pointers/references
  T const* operator->() const { return ptr.get(); }
  T const& operator*() const { return *ptr; }
};

template <typename T, typename... Args>
DeepConstSharedPtr<T> make_deep_const_shared(Args&&... args) {
  return DeepConstSharedPtr<T>(
      std::make_shared<T>(std::forward<Args>(args)...));
}
} // namespace cda_rail
