#include "StringHelper.hpp"

#include <cstddef>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>

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
std::string cda_rail::sanitize(std::string s) {
  // Replace all chars that are not isalnum with '-'
  std::ranges::replace_if(
      s, [](unsigned char const c) { return !std::isalnum(c); }, '-');
  return s;
}
