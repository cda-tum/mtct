#include <exception>
#include <string>
#include <utility>

namespace cda_rail::exceptions {
class VertexNotExistentException : public std::exception {
public:
  VertexNotExistentException()
      : error_message("Some vertex specified does not exist.") {}
  explicit VertexNotExistentException(const std::string& vertex_name)
      : error_message("Vertex " + vertex_name + " does not exist") {}
  explicit VertexNotExistentException(size_t vertex_id)
      : error_message("Vertex with ID " + std::to_string(vertex_id) +
                      " does not exist") {}
  [[nodiscard]] const char* what() const override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class EdgeNotExistentException : public std::exception {
public:
  EdgeNotExistentException()
      : error_message("Some edge specified does not exist.") {}
  explicit EdgeNotExistentException(const std::string& edge_name)
      : error_message("Edge " + edge_name + " does not exist.") {}
  explicit EdgeNotExistentException(size_t edge_id)
      : error_message("Edge with ID " + std::to_string(edge_id) +
                      " does not exist.") {}
  explicit EdgeNotExistentException(size_t source, size_t target)
      : error_message("Edge connecting vertices with IDs " +
                      std::to_string(source) + "->" + std::to_string(target) +
                      " does not exist.") {}
  explicit EdgeNotExistentException(const std::string& source,
                                    const std::string& target)
      : error_message("Edge connecting " + source + "->" + target +
                      " does not exist.") {}
  [[nodiscard]] const char* what() const override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};
} // namespace cda_rail::exceptions
