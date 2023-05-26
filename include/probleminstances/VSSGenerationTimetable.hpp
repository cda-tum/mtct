#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "datastructure/Route.hpp"

namespace cda_rail::instances{
    class VSSGenerationTimetable {
        private:
            cda_rail::Network network;
            cda_rail::Timetable timetable;
            cda_rail::RouteMap routes;
        public:
            // Constructors
            VSSGenerationTimetable() = default;
            VSSGenerationTimetable(const std::filesystem::path& p, bool every_train_must_have_route = true);
            VSSGenerationTimetable(const std::string& path, bool every_train_must_have_route = true) : VSSGenerationTimetable(std::filesystem::path(path), every_train_must_have_route) {};
            VSSGenerationTimetable(const char* path, bool every_train_must_have_route = true) : VSSGenerationTimetable(std::filesystem::path(path), every_train_must_have_route) {};

            // Rule of 5
            VSSGenerationTimetable(const VSSGenerationTimetable& other) = default;
            VSSGenerationTimetable(VSSGenerationTimetable&& other) = default;
            VSSGenerationTimetable& operator=(const VSSGenerationTimetable& other) = default;
            VSSGenerationTimetable& operator=(VSSGenerationTimetable&& other) = default;
            ~VSSGenerationTimetable() = default;

            // Network functions, i.e., network is accessible via n() as a reference
            [[nodiscard]] cda_rail::Network& n() {return network;};

            // Timetable functions
            int add_train(const std::string& name, int length, double max_speed, double acceleration, double deceleration,
                           int t_0, double v_0, int entry, int t_n, double v_n, int exit) {
                return timetable.add_train(name, length, max_speed, acceleration, deceleration, t_0, v_0, entry, t_n, v_n, exit, network);
            };
            int add_train(const std::string& name, int length, double max_speed, double acceleration, double deceleration,
                           int t_0, double v_0, const std::string& entry, int t_n, double v_n, const std::string& exit) {
                return timetable.add_train(name, length, max_speed, acceleration, deceleration, t_0, v_0, entry, t_n, v_n, exit, network);
            };

            void add_station(const std::string& name) {timetable.add_station(name);};

            void add_track_to_station(const std::string& name, int track) {timetable.add_track_to_station(name, track, network);};
            void add_track_to_station(const std::string& name, int source, int target) {timetable.add_track_to_station(name, source, target, network);};
            void add_track_to_station(const std::string& name, const std::string& source, const std::string& target) {timetable.add_track_to_station(name, source, target, network);};

            void add_stop(int train_index, const std::string& station_name, int begin, int end, bool sort = true) {
                timetable.add_stop(train_index, station_name, begin, end, sort);
            };
            void add_stop(const std::string& train_name, const std::string& station_name, int begin, int end, bool sort = true) {
                timetable.add_stop(train_name, station_name, begin, end, sort);
            };

            [[nodiscard]] const cda_rail::StationList& get_station_list() const {return timetable.get_station_list();};
            [[nodiscard]] const cda_rail::TrainList& get_train_list() const {return timetable.get_train_list();};
            [[nodiscard]] const Schedule& get_schedule(int index) const {return timetable.get_schedule(index);};
            [[nodiscard]] const Schedule& get_schedule(const std::string& train_name) const {return timetable.get_schedule(train_name);};

            [[nodiscard]] int maxT() const {return timetable.maxT();};
            [[nodiscard]] std::pair<int, int> time_interval(int train_index) const {return timetable.time_interval(train_index);};
            [[nodiscard]] std::pair<int, int> time_interval(const std::string& train_name) const {return timetable.time_interval(train_name);};

            void sort_stops() {timetable.sort_stops();};

            // RouteMap functions
            void add_empty_route(const std::string& train_name) {routes.add_empty_route(train_name, get_train_list());};

            void push_back_edge_to_route(const std::string& train_name, int edge_index) {routes.push_back_edge(train_name, edge_index, network);};
            void push_back_edge_to_route(const std::string& train_name, int source, int target) {routes.push_back_edge(train_name, source, target, network);};
            void push_back_edge_to_route(const std::string& train_name, const std::string& source, const std::string& target) {routes.push_back_edge(train_name, source, target, network);};

            void push_front_edge_to_route(const std::string& train_name, int edge_index) {routes.push_front_edge(train_name, edge_index, network);};
            void push_front_edge_to_route(const std::string& train_name, int source, int target) {routes.push_front_edge(train_name, source, target, network);};
            void push_front_edge_to_route(const std::string& train_name, const std::string& source, const std::string& target) {routes.push_front_edge(train_name, source, target, network);};

            void remove_first_edge_from_route(const std::string& train_name) {routes.remove_first_edge(train_name);};
            void remove_last_edge_from_route(const std::string& train_name) {routes.remove_last_edge(train_name);};

            [[nodiscard]] bool has_route(const std::string& train_name) const {return routes.has_route(train_name);};
            [[nodiscard]] int route_map_size() const {return routes.size();};
            [[nodiscard]] const Route& get_route(const std::string& train_name) const {return routes.get_route(train_name);};

            [[nodiscard]] double route_length(const std::string& train_name) const {return routes.length(train_name, network);};
            [[nodiscard]] std::pair<double, double> route_edge_pos(const std::string& train_name, int edge) const {
                return routes.edge_pos(train_name, edge, network);
            };
            [[nodiscard]] std::pair<double, double> route_edge_pos(const std::string& train_name, int source, int target) const {
                return routes.edge_pos(train_name, source, target, network);
            };
            [[nodiscard]] std::pair<double, double> route_edge_pos(const std::string& train_name, const std::string& source, const std::string& target) const {
                return routes.edge_pos(train_name, source, target, network);
            };
            [[nodiscard]] std::pair<double, double> route_edge_pos(const std::string& train_name, const std::vector<int>& edges) const {
                return routes.edge_pos(train_name, edges, network);
            };

            // General Consistency Check
            [[nodiscard]] bool check_consistency(bool every_train_must_have_route = true) const {
                return (timetable.check_consistency(network) &&
                        routes.check_consistency(get_train_list(), network, every_train_must_have_route));
            };

            // Export and import functions
            void export_instance(const std::filesystem::path& p) const;
            void export_instance(const std::string& path) const {export_instance(std::filesystem::path(path));};
            void export_instance(const char* path) const {export_instance(std::filesystem::path(path));};

            [[nodiscard]] static VSSGenerationTimetable import_instance(const std::filesystem::path& p, bool every_train_must_have_route = true) {
                return VSSGenerationTimetable(p, every_train_must_have_route);
            };
            [[nodiscard]] static VSSGenerationTimetable import_instance(const std::string& path, bool every_train_must_have_route = true) {
                return VSSGenerationTimetable(path, every_train_must_have_route);
            };
            [[nodiscard]] static VSSGenerationTimetable import_instance(const char* path, bool every_train_must_have_route = true) {
                return VSSGenerationTimetable(path, every_train_must_have_route);
            };

            // Transformation functions
            void discretize(cda_rail::SeparationType separation_type = cda_rail::SeparationType::UNIFORM);

            // Helper
            std::vector<int> trains_in_section(const std::vector<int>& section) const;
            std::vector<int> trains_at_t(int t) const;
            std::vector<int> trains_at_t(int t, const std::vector<int>& trains_to_consider) const;
    };
}