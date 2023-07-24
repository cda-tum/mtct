#include <exception>
#include <string>

namespace cda_rail::exceptions {
class vertex_not_existent_exception : public std::exception {
public:
  vertex_not_existent_exception() : vertex_name(" passed to function ") {}
  vertex_not_existent_exception(const std::string& vertex_name)
      : vertex_name(vertex_name) {}
  vertex_not_existent_exception(size_t vertex_id)
      : vertex_name(std::to_string(vertex_id)) {}
  const char* what() const noexcept override {
    return ("Vertex " + vertex_name + " does not exist").c_str();
  }

private:
  std::string vertex_name;
};

class edge_not_existent_exception : public std::exception {
public:
  edge_not_existent_exception() : edge_name(" passed to function ") {}
  edge_not_existent_exception(const std::string& edge_name)
      : edge_name(edge_name) {}
  edge_not_existent_exception(size_t edge_id)
      : edge_name(std::to_string(edge_id)) {}
  edge_not_existent_exception(size_t source, size_t target)
      : edge_name(std::to_string(source) + " -> " + std::to_string(target)) {}
  edge_not_existent_exception(const std::string& source,
                              const std::string& target)
      : edge_name(source + " -> " + target) {}
  const char* what() const noexcept override {
    return ("Edge " + edge_name + " does not exist").c_str();
  }

private:
  std::string edge_name;
};
} // namespace cda_rail::exceptions
