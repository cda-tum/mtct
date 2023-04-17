#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Train.hpp"
#include <vector>
#include <string>
#include <filesystem>
#include <unordered_map>

namespace cda_rail {
    class Route {
        private:
            std::vector<int> edges;
        public:
            void push_back_edge(int edge_index, cda_rail::Network network);
            void push_back_edge(int source, int target, cda_rail::Network network);
            void push_back_edge(const std::string& source, const std::string& target, cda_rail::Network network);

            void push_front_edge(int edge_index, cda_rail::Network network);
            void push_front_edge(int source, int target, cda_rail::Network network);
            void push_front_edge(const std::string& source, const std::string& target, cda_rail::Network network);

            void remove_first_edge();
            void remove_last_edge();

            [[nodiscard]] int get_edge(int route_index) const;
            [[nodiscard]] const cda_rail::Edge& get_edge(int route_index, cda_rail::Network network) const;
            [[nodiscard]] int size() const;

            [[nodiscard]] bool assert(cda_rail::Network& network) const;
    };

    class RouteMap {
        private:
            std::unordered_map<std::string, Route> routes;
        public:
            void add_empty_route(const std::string& train_name);
            void add_empty_route(const std::string& train_name, cda_rail::TrainList& trains);

            template<typename... Ts>
            void push_back_edge(const std::string& train_name, Ts&&... args);

            template<typename... Ts>
            void push_front_edge(const std::string& train_name, Ts&&... args);

            void remove_first_edge(const std::string& train_name);
            void remove_last_edge(const std::string& train_name);

            const Route& get_route(const std::string& train_name) const;

            [[nodiscard]] bool assert(cda_rail::TrainList& trains, cda_rail::Network& network) const;

            void export_routes(const std::filesystem::path& p, cda_rail::Network& network) const;
            void export_routes(const std::string& path, cda_rail::Network& network) const;
            void export_routes(const char* path, cda_rail::Network& network) const;

            [[nodiscard]] static RouteMap import_routes(const std::filesystem::path& p, cda_rail::Network& network);
            [[nodiscard]] static RouteMap import_routes(const std::string& path, cda_rail::Network& network);
            [[nodiscard]] static RouteMap import_routes(const char* path, cda_rail::Network& network);
    };
}