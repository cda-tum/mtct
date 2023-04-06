#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include <vector>
#include <string>
#include <filesystem>

namespace cda_rail {
    class Route {
        private:
            std::vector<int> edges;
        public:
            void push_back_edge(int edge_index, cda_rail::Network network);
            template<typename T>
            void push_back_edge(T source, T target, cda_rail::Network network);

            void push_front_edge(int edge_index, cda_rail::Network network);
            template<typename T>
            void push_front_edge(T source, T target, cda_rail::Network network);

            void remove_edge(int edge_index, cda_rail::Network network);
            template<typename T>
            void remove_edge(T source, T target, cda_rail::Network network);

            void export_route(const std::filesystem::path& p, cda_rail::Network network) const;
            void export_route(const std::string& path, cda_rail::Network network) const;
            void export_route(const char* path, cda_rail::Network network) const;

            void import_route(const std::filesystem::path& p, cda_rail::Network network);
            void import_route(const std::string& path, cda_rail::Network network);
            void import_route(const char* path, cda_rail::Network network);
    };
}