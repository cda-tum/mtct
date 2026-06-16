#include "StringHelper.hpp"

#include <cstddef>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>

/**
 * @brief Concatenates multiple string views into a single string.
 *
 * @return std::string The concatenated result of all input string views.
 *
 * @example
 * std::string result = concatenate_string_views({"hello", " ", "world"});
 * // result is "hello world"
 */

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

/**
 * @brief Parses a string view as a boolean and updates an optional if recognized.
 *
 * Compares @p sView against "true" and "false" in a case-insensitive manner. Sets @p b to `true` if
 * @p sView matches "true", or to `false` if @p sView matches "false". If @p sView matches neither,
 * @p b remains unchanged.
 *
 * @param sView The string to parse.
 * @param b Reference to the optional boolean to update.
 */
void cda_rail::to_bool_optional_inplace(std::string_view const sView,
                                        std::optional<bool>&   b) {
  if (case_insensitive_str_equal(sView, "true")) {
    b = true;
  } else if (case_insensitive_str_equal(sView, "false")) {
    b = false;
  }
}
