#pragma once

#include <algorithm>
#include <cctype>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>

namespace cda_rail {

/**
 * @brief Concatenates multiple string views into a single string.
 * @param parts String fragments to concatenate in the given order.
 * @return String containing all fragments from @p parts appended in order.
 * The length of the returned string equals the sum of the fragment lengths.
 */
[[nodiscard]] std::string
concatenate_string_views(std::initializer_list<std::string_view> parts);

// Helper for allocation-free, case-insensitive comparison
inline bool case_insensitive_str_equal(std::string_view a, std::string_view b) {
  return std::equal(a.begin(), a.end(), b.begin(), b.end(),
                    [](char const c1, char const c2) {
                      // Cast to unsigned char to avoid UB with extended ASCII
                      return std::tolower(static_cast<unsigned char>(c1)) ==
                             std::tolower(static_cast<unsigned char>(c2));
                    });
}

void to_bool_optional_inplace(std::string_view sView, std::optional<bool>& b);

} // namespace cda_rail
