#include "StringHelper.hpp"

#include <cstddef>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>

// String Helper

std::string cda_rail::concatenate_string_views(
    std::initializer_list<std::string_view> parts) {
  std::size_t total_size = 0;
  for (std::string_view const part : parts) {
    total_size += part.size();
  }

  std::string result;
  result.reserve(total_size);
  for (std::string_view const part : parts) {
    result.append(part);
  }
  return result;
}

void cda_rail::to_bool_optional_inplace(std::string_view const sView,
                                        std::optional<bool>&   b) {
  if (case_insensitive_str_equal(sView, "true")) {
    b = true;
  } else if (case_insensitive_str_equal(sView, "false")) {
    b = false;
  }
}
