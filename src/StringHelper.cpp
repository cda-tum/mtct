#include "StringHelper.hpp"

#include <algorithm>
#include <optional>
#include <ranges>

// String Helper

std::string cda_rail::concatenate_string_views(
    std::initializer_list<std::string_view> parts) {
  const auto total_size = std::ranges::fold_left(
      parts | std::views::transform(&std::string_view::size), std::size_t{0},
      std::plus{});

  std::string result;
  result.reserve(total_size);
  for (std::string_view const part : parts) {
    result.append(part);
  }
  return result;
}

void cda_rail::to_bool_optional_inplace(std::string_view const s_view,
                                        std::optional<bool>&   b) {
  if (case_insensitive_str_equal(s_view, "true")) {
    b = true;
  } else if (case_insensitive_str_equal(s_view, "false")) {
    b = false;
  }
}
