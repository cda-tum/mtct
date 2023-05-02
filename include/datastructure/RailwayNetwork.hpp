#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
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
    };

    class Network {
        /**
         * Graph class
         *
         */
        private:
            std::vector<Vertex> vertices;
            std::vector<Edge> edges;
            std::vector<std::unordered_set<int>> successors;
            std::unordered_map<std::string, int> vertex_name_to_index;

            [[nodiscard]] static Network read_graphml(const std::filesystem::path& p);
            static void get_keys(tinyxml2::XMLElement* graphml_body, std::string& breakable, std::string& length, std::string& max_speed, std::string& min_block_length, std::string& type);
            static void add_vertices_from_graphml(const tinyxml2::XMLElement* graphml_node, cda_rail::Network& network, const std::string& type);
            static void add_edges_from_graphml(const tinyxml2::XMLElement* graphml_edge, cda_rail::Network& network, const std::string& breakable, const std::string& length, const std::string& max_speed, const std::string& min_block_length);
            void read_successors(const std::filesystem::path& p);
            static void extract_vertices_from_key(const std::string& key, std::string& source_name, std::string& target_name);

            void export_graphml(const std::filesystem::path& p) const;
            void export_successors_python(const std::filesystem::path& p) const;
            void export_successors_cpp(const std::filesystem::path& p) const;
            void write_successor_set_to_file(std::ofstream& file, int i) const;
        public:
            int add_vertex(const std::string& name, VertexType type);
            int add_edge(int source,int target, double length, double max_speed, bool breakable, double min_block_length = 0);
            int add_edge(const std::string& source_name, const std::string& target_name, double length, double max_speed, bool breakable, double min_block_length = 0);
            void add_successor(int edge_in, int edge_out);
            void add_successor(const std::pair<int, int>& edge_in, const std::pair<int, int>& edge_out);
            void add_successor(const std::pair<std::string, std::string>& edge_in, const std::pair<std::string, std::string>& edge_out);
            void add_successor(int edge_in, const std::pair<int, int>& edge_out);
            void add_successor(int edge_in, const std::pair<std::string, std::string>& edge_out);
            void add_successor(const std::pair<int, int>& edge_in, int edge_out);
            void add_successor(const std::pair<int, int>& edge_in, const std::pair<std::string, std::string>& edge_out);
            void add_successor(const std::pair<std::string, std::string>& edge_in, int edge_out);
            void add_successor(const std::pair<std::string, std::string>& edge_in, const std::pair<int, int>& edge_out);


            [[nodiscard]] const Vertex& get_vertex(int index) const;
            [[nodiscard]] const Vertex& get_vertex(const std::string& name) const;
            [[nodiscard]] int get_vertex_index(const std::string& name) const;
            [[nodiscard]] const Edge& get_edge(int index) const;
            [[nodiscard]] const Edge& get_edge(int source_id, int target_id) const;
            [[nodiscard]] const Edge& get_edge(const std::string& source_name, const std::string& target_name) const;
            [[nodiscard]] int get_edge_index(int source_id, int target_id) const;
            [[nodiscard]] int get_edge_index(const std::string& source_name, const std::string& target_name) const;

            [[nodiscard]] bool has_vertex(int index) const;
            [[nodiscard]] bool has_vertex(const std::string& name) const;
            [[nodiscard]] bool has_edge(int index) const;
            [[nodiscard]] bool has_edge(int source_id, int target_id) const;
            [[nodiscard]] bool has_edge(const std::string& source_name, const std::string& target_name) const;

            void change_vertex_name(int index, const std::string& new_name);
            void change_vertex_name(const std::string& old_name, const std::string& new_name);

            void change_edge_property(int index, double value, const std::string& property);
            void change_edge_property(int source_id, int target_id, double value, const std::string& property);
            void change_edge_property(const std::string& source_name, const std::string& target_name, double value, const std::string& property);
            void change_edge_breakable(int index, bool value);
            void change_edge_breakable(int source_id, int target_id, bool value);
            void change_edge_breakable(const std::string& source_name, const std::string& target_name, bool value);

            [[nodiscard]] std::unordered_set<int> out_edges(int index) const;
            [[nodiscard]] std::unordered_set<int> out_edges(const std::string& name) const;
            [[nodiscard]] std::unordered_set<int> in_edges(int index) const;
            [[nodiscard]] std::unordered_set<int> in_edges(const std::string& name) const;
            [[nodiscard]] const std::unordered_set<int>& get_successors(int index) const;
            [[nodiscard]] const std::unordered_set<int>& get_successors(int source_id, int target_id) const;
            [[nodiscard]] const std::unordered_set<int>& get_successors(const std::string& source_name, const std::string& target_name) const;

            [[nodiscard]] std::unordered_set<int> neighbors(int index) const;
            [[nodiscard]] std::unordered_set<int> neighbors(const std::string& name) const;

            [[nodiscard]] int number_of_vertices() const;
            [[nodiscard]] int number_of_edges() const;

            [[nodiscard]] static Network import_network(const std::string& path);
            [[nodiscard]] static Network import_network(const char* path);
            [[nodiscard]] static Network import_network(const std::filesystem::path& p);
            void export_network(const std::string& path) const;
            void export_network(const char*) const;
            void export_network(const std::filesystem::path& p) const;

            [[nodiscard]] bool is_valid_successor(int e0, int e1) const;
    };

    // HELPER
    void to_bool_optional(std::string &s, std::optional<bool> &b);
}
