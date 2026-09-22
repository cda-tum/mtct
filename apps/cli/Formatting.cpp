#include "Formatting.hpp"

#include "Definitions.hpp"

#include <iomanip>
#include <ios>
#include <map>
#include <sstream>
#include <string>

const std::map<std::string, cda_rail::VertexType>&
cda_rail::cli::vertex_type_map() {
  static const std::map<std::string, VertexType> map{
      {"NoBorder", VertexType::NoBorder},
      {"VSS", VertexType::VSS},
      {"TTD", VertexType::TTD},
      {"NoBorderVSS", VertexType::NoBorderVSS}};
  return map;
}

std::string cda_rail::cli::vertex_type_to_string(VertexType const type) {
  return key_by_value(vertex_type_map(), type);
}

std::string cda_rail::cli::format_double(double const value) {
  std::stringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  std::string str = stream.str();
  str.erase(str.find_last_not_of('0') + 1, std::string::npos);
  if (str.back() == '.') {
    str.pop_back();
  }
  return str;
}
