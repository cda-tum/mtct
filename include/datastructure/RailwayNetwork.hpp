#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <sstream>
#include <algorithm>
#include <tinyxml2.h>
#include <filesystem>
#include "Definitions.hpp"

namespace cda_rail {
    struct Vertex {
        /**
         * Vertex object
         * @param name Name of the vertex
         * @param type Type of the vertex (NO_BORDER, VSS, TTD)
         */

        std::string name;
        VertexType type;

        // Constructors
        Vertex() = default;
        Vertex(const std::string& name, VertexType type) : name(name), type(type) {};
    };

    struct Edge {
        /**
         * Edge object
         * @param source Source vertex index
         * @param target Target vertex index
         * @param length Length of vertex (in m)
         * @param max_speed Speed limit of vertex (in m/s)
         * @param breakable Boolean indicating if VSS can be placed on this edge
         * @param min_block_length Minimum block length (in m). Optional, default 0.
         */
        int source;
        int target;
        double length;
        double max_speed;
        bool breakable;
        double min_block_length = 0;

        // Constructors
        Edge() = default;
        Edge(int source, int target, double length, double max_speed, bool breakable, double min_block_length = 0) :
            source(source), target(target), length(length), max_speed(max_speed), breakable(breakable), min_block_length(min_block_length) {};
    };

    class Network {
        /**
         * Graph class
         *
         */
        private:
            std::vector<Vertex> vertices;
            std::vector<Edge> edges;
            std::vector<std::vector<int>> successors;
            std::unordered_map<std::string, int> vertex_name_to_index;

            void read_graphml(const std::filesystem::path& p);
            static void get_keys(tinyxml2::XMLElement* graphml_body, std::string& breakable, std::string& length, std::string& max_speed, std::string& min_block_length, std::string& type);
            void add_vertices_from_graphml(const tinyxml2::XMLElement* graphml_node, const std::string& type);
            void add_edges_from_graphml(const tinyxml2::XMLElement* graphml_edge, const std::string& breakable, const std::string& length, const std::string& max_speed, const std::string& min_block_length);
            void read_successors(const std::filesystem::path& p);
            static void extract_vertices_from_key(const std::string& key, std::string& source_name, std::string& target_name);

            void export_graphml(const std::filesystem::path& p) const;
            void export_successors_python(const std::filesystem::path& p) const;
            void export_successors_cpp(const std::filesystem::path& p) const;
            void write_successor_set_to_file(std::ofstream& file, int i) const;
        public:
            // Constructors
            Network() = default;
            Network(const std::filesystem::path& p);
            Network(const std::string& path) : Network(std::filesystem::path(path)) {};
            Network(const char* path) : Network(std::filesystem::path(path)) {};

            // Rule of 5
            Network(const Network& other) = default;
            Network(Network&& other) noexcept = default;
            Network& operator=(const Network& other) = default;
            Network& operator=(Network&& other) noexcept = default;
            ~Network() = default;

            int add_vertex(const std::string& name, VertexType type);
            int add_edge(int source,int target, double length, double max_speed, bool breakable, double min_block_length = 0);
            int add_edge(const std::string& source_name, const std::string& target_name, double length, double max_speed, bool breakable, double min_block_length = 0) {
                return add_edge(get_vertex_index(source_name), get_vertex_index(target_name), length,
                                max_speed, breakable, min_block_length);
            };
            void add_successor(int edge_in, int edge_out);
            void add_successor(const std::pair<int, int>& edge_in, const std::pair<int, int>& edge_out) {
                add_successor(get_edge_index(edge_in.first, edge_in.second), get_edge_index(edge_out.first, edge_out.second));
            };
            void add_successor(const std::pair<std::string, std::string>& edge_in, const std::pair<std::string, std::string>& edge_out) {
                add_successor(get_edge_index(edge_in.first, edge_in.second), get_edge_index(edge_out.first, edge_out.second));
            };
            void add_successor(int edge_in, const std::pair<int, int>& edge_out) {
                add_successor(edge_in, get_edge_index(edge_out.first, edge_out.second));
            };
            void add_successor(int edge_in, const std::pair<std::string, std::string>& edge_out) {
                add_successor(edge_in, get_edge_index(edge_out.first, edge_out.second));
            };
            void add_successor(const std::pair<int, int>& edge_in, int edge_out) {
                add_successor(get_edge_index(edge_in.first, edge_in.second), edge_out);
            };
            void add_successor(const std::pair<int, int>& edge_in, const std::pair<std::string, std::string>& edge_out) {
                add_successor(get_edge_index(edge_in.first, edge_in.second), get_edge_index(edge_out.first, edge_out.second));
            };
            void add_successor(const std::pair<std::string, std::string>& edge_in, int edge_out) {
                add_successor(get_edge_index(edge_in.first, edge_in.second), edge_out);
            };
            void add_successor(const std::pair<std::string, std::string>& edge_in, const std::pair<int, int>& edge_out) {
                add_successor(get_edge_index(edge_in.first, edge_in.second), get_edge_index(edge_out.first, edge_out.second));
            };


            [[nodiscard]] const Vertex& get_vertex(int index) const;
            [[nodiscard]] const Vertex& get_vertex(const std::string& name) const {return vertices[get_vertex_index(name)];};
            [[nodiscard]] int get_vertex_index(const std::string& name) const;
            [[nodiscard]] const Edge& get_edge(int index) const;
            [[nodiscard]] const Edge& get_edge(int source_id, int target_id) const;
            [[nodiscard]] const Edge& get_edge(const std::string& source_name, const std::string& target_name) const {
                return get_edge(get_vertex_index(source_name), get_vertex_index(target_name));
            };
            [[nodiscard]] int get_edge_index(int source_id, int target_id) const;
            [[nodiscard]] int get_edge_index(const std::string& source_name, const std::string& target_name) const {
                return get_edge_index(get_vertex_index(source_name), get_vertex_index(target_name));
            };

            [[nodiscard]] bool has_vertex(int index) const {return (index >= 0 && index < vertices.size());};
            [[nodiscard]] bool has_vertex(const std::string& name) const {return vertex_name_to_index.find(name) != vertex_name_to_index.end();};
            [[nodiscard]] bool has_edge(int index) const {return (index >= 0 && index < edges.size());};
            [[nodiscard]] bool has_edge(int source_id, int target_id) const;
            [[nodiscard]] bool has_edge(const std::string& source_name, const std::string& target_name) const;

            void change_vertex_name(int index, const std::string& new_name);
            void change_vertex_name(const std::string& old_name, const std::string& new_name) {change_vertex_name(get_vertex_index(old_name), new_name);};

            void change_edge_property(int index, double value, const std::string& property);
            void change_edge_property(int source_id, int target_id, double value, const std::string& property) {change_edge_property(get_edge_index(source_id, target_id), value, property);};
            void change_edge_property(const std::string& source_name, const std::string& target_name, double value, const std::string& property) {
                change_edge_property(get_edge_index(source_name, target_name), value, property);
            };
            void change_edge_breakable(int index, bool value);
            void change_edge_breakable(int source_id, int target_id, bool value) {change_edge_breakable(get_edge_index(source_id, target_id), value);};
            void change_edge_breakable(const std::string& source_name, const std::string& target_name, bool value) {change_edge_breakable(get_edge_index(source_name, target_name), value);};

            [[nodiscard]] std::vector<int> out_edges(int index) const;
            [[nodiscard]] std::vector<int> out_edges(const std::string& name) const {return out_edges(get_vertex_index(name));};
            [[nodiscard]] std::vector<int> in_edges(int index) const;
            [[nodiscard]] std::vector<int> in_edges(const std::string& name) const {return in_edges(get_vertex_index(name));};
            [[nodiscard]] const std::vector<int>& get_successors(int index) const;
            [[nodiscard]] const std::vector<int>& get_successors(int source_id, int target_id) const {return get_successors(get_edge_index(source_id, target_id));};
            [[nodiscard]] const std::vector<int>& get_successors(const std::string& source_name, const std::string& target_name) const {
                return get_successors(get_edge_index(source_name, target_name));
            };

            [[nodiscard]] std::vector<int> neighbors(int index) const;
            [[nodiscard]] std::vector<int> neighbors(const std::string& name) const {return neighbors(get_vertex_index(name));};

            [[nodiscard]] int number_of_vertices() const {return vertices.size();};
            [[nodiscard]] int number_of_edges() const {return edges.size();};

            [[nodiscard]] static Network import_network(const std::string& path) {return Network(path);};
            [[nodiscard]] static Network import_network(const char* path) {return Network(path);};
            [[nodiscard]] static Network import_network(const std::filesystem::path& p) {return Network(p);};
            void export_network(const std::string& path) const {export_network(std::filesystem::path(path));};
            void export_network(const char* path) const {export_network(std::filesystem::path(path));};
            void export_network(const std::filesystem::path& p) const;

            [[nodiscard]] bool is_valid_successor(int e0, int e1) const;
    };

    // HELPER
    void to_bool_optional(std::string &s, std::optional<bool> &b);
}
