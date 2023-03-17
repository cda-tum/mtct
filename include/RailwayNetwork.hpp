#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace cda_rail {
    struct Vertex {
        /**
         * Vertex object
         * @param name Name of the vertex
         * @param type Type of the vertex (NO_BORDER, VSS, TTD)
         */

        std::string name;
        int type;
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
        public:
            void add_vertex(const std::string& name, const int type);
            void add_edge(const int source, const int target, const double length, const double max_speed, const bool breakable, const double min_block_length = 0);
            void add_edge(const std::string& source_name, const std::string& target_name, const double length, const double max_speed, const bool breakable, const double min_block_length = 0);
            void add_successor(const int edge_in, const int edge_out);

            [[nodiscard]] const Vertex& get_vertex(const int index) const;
            [[nodiscard]] const Vertex& get_vertex(const std::string& name) const;
            [[nodiscard]] int get_vertex_index(const std::string& name) const;
            [[nodiscard]] const Edge& get_edge(const int index) const;
            [[nodiscard]] const Edge& get_edge(const int source_id, const int target_id) const;
            [[nodiscard]] const Edge& get_edge(const std::string& source_name, const std::string& target_name) const;
            [[nodiscard]] int get_edge_index(const int source_id, const int target_id) const;
            [[nodiscard]] int get_edge_index(const std::string& source_name, const std::string& target_name) const;

            [[nodiscard]] bool has_vertex(const int index) const;
            [[nodiscard]] bool has_vertex(const std::string& name) const;
            [[nodiscard]] bool has_edge(const int index) const;
            [[nodiscard]] bool has_edge(const int source_id, const int target_id) const;
            [[nodiscard]] bool has_edge(const std::string& source_name, const std::string& target_name) const;

            void change_vertex_name(const int index, const std::string& new_name);
            void change_vertex_name(const std::string& old_name, const std::string& new_name);

            void change_edge_property(const int index, const double value, const std::string& property);
            void change_edge_property(const int source_id, const int target_id, const double value, const std::string& property);
            void change_edge_property(const std::string& source_name, const std::string& target_name, const double value, const std::string& property);
            void change_edge_breakable(const int index, const bool value);
            void change_edge_breakable(const int source_id, const int target_id, const bool value);
            void change_edge_breakable(const std::string& source_name, const std::string& target_name, const bool value);

            [[nodiscard]] const std::unordered_set<int>& out_edges(const int index) const;
            [[nodiscard]] const std::unordered_set<int>& out_edges(const std::string& name) const;
            [[nodiscard]] const std::unordered_set<int>& in_edges(const int index) const;
            [[nodiscard]] const std::unordered_set<int>& in_edges(const std::string& name) const;

            [[nodiscard]] static Network read_network(const std::string& path);
    };
}
