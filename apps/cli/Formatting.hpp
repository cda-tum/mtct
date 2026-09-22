#pragma once

#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"

#include <map>
#include <string>
#include <string_view>

namespace cda_rail::cli {

/**
 * @brief Wraps a command argument as a vertex descriptor.
 *
 * `Network::VertexInput` is implicitly constructible from a `std::string_view`
 * but not from a `std::string`, since that would need two user-defined
 * conversions. The descriptor refers to @p name, which therefore has to
 * outlive the call it is passed to.
 */
[[nodiscard]] inline Network::VertexInput
vertex_input(const std::string& name) {
  return {std::string_view{name}};
}
/** @brief Wraps two command arguments as an edge descriptor, see above. */
[[nodiscard]] inline Network::EdgeInput edge_input(const std::string& source,
                                                   const std::string& target) {
  return {std::string_view{source}, std::string_view{target}};
}

/** @brief Names accepted for `--type` and printed by the list commands. */
[[nodiscard]] const std::map<std::string, VertexType>& vertex_type_map();
/** @brief The name of @p type, as the list commands print it. */
[[nodiscard]] std::string vertex_type_to_string(VertexType type);

/**
 * @brief Formats a double without trailing zeros.
 *
 * Used both for the printed listings and for the generated parameter
 * identifiers, which end up as directory names.
 */
[[nodiscard]] std::string format_double(double value);

/** @brief `"yes"`/`"no"`, as the solver apps print their flags. */
[[nodiscard]] inline const char* yes_no(bool value) {
  return value ? "yes" : "no";
}
/** @brief `"t"`/`"f"`, as used inside generated parameter identifiers. */
[[nodiscard]] inline const char* bool_to_str(bool value) {
  return value ? "t" : "f";
}

/**
 * @brief Looks a value up in one of the option maps to print it by its name.
 *
 * The option maps are name-to-value, since that is what CLI11's
 * `CheckedTransformer` expects; printing needs the reverse direction.
 */
template <typename Map, typename Value>
[[nodiscard]] std::string key_by_value(const Map& map, const Value& value) {
  for (const auto& [key, val] : map) {
    if (val == value) {
      return key;
    }
  }
  return "unknown";
}

} // namespace cda_rail::cli
