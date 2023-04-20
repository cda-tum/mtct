#include "probleminstances/VSSGenerationTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"

cda_rail::Network &cda_rail::instances::VSSGenerationTimetable::n() {
    return network;
}

void cda_rail::instances::VSSGenerationTimetable::add_train(const std::string &name, int length, double max_speed,
                                                            double acceleration, double deceleration, int t_0,
                                                            double v_0, int entry, int t_n, double v_n, int exit) {
    timetable.add_train(name, length, max_speed, acceleration, deceleration, t_0, v_0, entry, t_n, v_n, exit, network);
}

void cda_rail::instances::VSSGenerationTimetable::add_train(const std::string &name, int length, double max_speed,
                                                            double acceleration, double deceleration, int t_0,
                                                            double v_0, const std::string &entry, int t_n, double v_n,
                                                            const std::string &exit) {
    timetable.add_train(name, length, max_speed, acceleration, deceleration, t_0, v_0, entry, t_n, v_n, exit, network);
}

void cda_rail::instances::VSSGenerationTimetable::add_stop(int train_index, int station_index, int begin, int end,
                                                           bool sort) {
    timetable.add_stop(train_index, station_index, begin, end, sort);
}

void cda_rail::instances::VSSGenerationTimetable::add_stop(const std::string &train_name, int station_index, int begin,
                                                           int end, bool sort) {
    timetable.add_stop(train_name, station_index, begin, end, sort);
}

void cda_rail::instances::VSSGenerationTimetable::add_stop(int train_index, const std::string &station_name, int begin,
                                                           int end, bool sort) {
    timetable.add_stop(train_index, station_name, begin, end, sort);
}

void
cda_rail::instances::VSSGenerationTimetable::add_stop(const std::string &train_name, const std::string &station_name,
                                                      int begin, int end, bool sort) {
    timetable.add_stop(train_name, station_name, begin, end, sort);
}

const cda_rail::StationList &cda_rail::instances::VSSGenerationTimetable::get_station_list() const {
    return timetable.get_station_list();
}

const cda_rail::TrainList &cda_rail::instances::VSSGenerationTimetable::get_train_list() const {
    return timetable.get_train_list();
}

const cda_rail::Schedule &cda_rail::instances::VSSGenerationTimetable::get_schedule(int index) const {
    return timetable.get_schedule(index);
}

const cda_rail::Schedule &
cda_rail::instances::VSSGenerationTimetable::get_schedule(const std::string &train_name) const {
    return timetable.get_schedule(train_name);
}

void cda_rail::instances::VSSGenerationTimetable::sort_stops() {
    timetable.sort_stops();
}

void cda_rail::instances::VSSGenerationTimetable::add_empty_route(const std::string &train_name) {
    routes.add_empty_route(train_name, get_train_list());
}

void
cda_rail::instances::VSSGenerationTimetable::push_back_edge_to_route(const std::string &train_name, int edge_index) {
    routes.push_back_edge(train_name, edge_index, network);
}

void cda_rail::instances::VSSGenerationTimetable::push_back_edge_to_route(const std::string &train_name, int source,
                                                                          int target) {
    routes.push_back_edge(train_name, source, target, network);
}

void cda_rail::instances::VSSGenerationTimetable::push_back_edge_to_route(const std::string &train_name,
                                                                          const std::string &source,
                                                                          const std::string &target) {
    routes.push_back_edge(train_name, source, target, network);
}

void
cda_rail::instances::VSSGenerationTimetable::push_front_edge_to_route(const std::string &train_name, int edge_index) {
    routes.push_front_edge(train_name, edge_index, network);
}

void cda_rail::instances::VSSGenerationTimetable::push_front_edge_to_route(const std::string &train_name, int source,
                                                                           int target) {
    routes.push_front_edge(train_name, source, target, network);
}

void cda_rail::instances::VSSGenerationTimetable::push_front_edge_to_route(const std::string &train_name,
                                                                           const std::string &source,
                                                                           const std::string &target) {
    routes.push_front_edge(train_name, source, target, network);
}

void cda_rail::instances::VSSGenerationTimetable::remove_first_edge_from_route(const std::string &train_name) {
    routes.remove_first_edge(train_name);
}

void cda_rail::instances::VSSGenerationTimetable::remove_last_edge_from_route(const std::string &train_name) {
    routes.remove_last_edge(train_name);
}

bool cda_rail::instances::VSSGenerationTimetable::has_route(const std::string &train_name) const {
    return routes.has_route(train_name);
}

int cda_rail::instances::VSSGenerationTimetable::route_map_size() const {
    return routes.size();
}

const cda_rail::Route &cda_rail::instances::VSSGenerationTimetable::get_route(const std::string &train_name) const {
    return routes.get_route(train_name);
}

bool cda_rail::instances::VSSGenerationTimetable::check_consistency(bool every_train_must_have_route) const {
    return (timetable.check_consistency(network) &&
        routes.check_consistency(get_train_list(), network, every_train_must_have_route));
}

template<typename... Args>
void cda_rail::instances::VSSGenerationTimetable::add_track_to_station(Args &&... args) {
    timetable.add_station(args..., network);
}

template<typename... Args>
void cda_rail::instances::VSSGenerationTimetable::add_station(Args &&... args) {
    timetable.add_station(args...);
}
