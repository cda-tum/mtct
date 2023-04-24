#include "probleminstances/VSSGenerationTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "Definitions.hpp"

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

void cda_rail::instances::VSSGenerationTimetable::export_instance(const std::filesystem::path &p) const {
    /**
     * Exports the instance to the given path, i.e.,
     * - the network into the folder "network"
     * - the timetable into the folder "timetable"
     * - the routes into the folder "routes"
     *
     * @param p the path to the folder where the instance should be exported
     */

    if (!cda_rail::is_directory_and_create(p)) {
        throw std::runtime_error("Could not create directory " + p.string());
    }
    network.export_network(p / "network");
    timetable.export_timetable(p / "timetable", network);
    routes.export_routes(p / "routes", network);
}

void cda_rail::instances::VSSGenerationTimetable::export_instance(const std::string &path) const {
    std::filesystem::path p(path);
    export_instance(p);
}

void cda_rail::instances::VSSGenerationTimetable::export_instance(const char *path) const {
    std::filesystem::path p(path);
    export_instance(p);
}

cda_rail::instances::VSSGenerationTimetable
cda_rail::instances::VSSGenerationTimetable::import_instance(const std::filesystem::path &p, bool every_train_must_have_route) {
    /**
     * Imports an instance from the given path, i.e.,
     * - the network from the folder "network"
     * - the timetable from the folder "timetable"
     * - the routes from the folder "routes"
     *
     * @param p the path to the folder where the instance should be imported from
     */

    cda_rail::instances::VSSGenerationTimetable instance;
    instance.network = cda_rail::Network::import_network(p / "network");
    instance.timetable = cda_rail::Timetable::import_timetable(p / "timetable", instance.network);
    instance.routes = cda_rail::RouteMap::import_routes(p / "routes", instance.network);
    if (!instance.check_consistency(every_train_must_have_route)) {
        throw std::runtime_error("The imported instance is not consistent.");
    }
    return instance;
}

cda_rail::instances::VSSGenerationTimetable
cda_rail::instances::VSSGenerationTimetable::import_instance(const std::string &path,
                                                             bool every_train_must_have_route) {
    std::filesystem::path p(path);
    return import_instance(p, every_train_must_have_route);
}

cda_rail::instances::VSSGenerationTimetable
cda_rail::instances::VSSGenerationTimetable::import_instance(const char *path, bool every_train_must_have_route) {
    std::filesystem::path p(path);
    return import_instance(p, every_train_must_have_route);
}

void cda_rail::instances::VSSGenerationTimetable::add_station(const std::string &name,
                                                              const std::unordered_set<int> &tracks) {
    timetable.add_station(name, tracks);
}

void cda_rail::instances::VSSGenerationTimetable::add_station(const std::string &name) {
    timetable.add_station(name);
}

void cda_rail::instances::VSSGenerationTimetable::add_track_to_station(int station_index, int track) {
    timetable.add_track_to_station(station_index, track, network);
}

void cda_rail::instances::VSSGenerationTimetable::add_track_to_station(const std::string &name, int track) {
    timetable.add_track_to_station(name, track, network);
}

void cda_rail::instances::VSSGenerationTimetable::add_track_to_station(int station_index, int source, int target) {
    timetable.add_track_to_station(station_index, source, target, network);
}

void
cda_rail::instances::VSSGenerationTimetable::add_track_to_station(const std::string &name, int source, int target) {
    timetable.add_track_to_station(name, source, target, network);
}

void cda_rail::instances::VSSGenerationTimetable::add_track_to_station(int station_index, const std::string &source,
                                                                       const std::string &target) {
    timetable.add_track_to_station(station_index, source, target, network);
}

void
cda_rail::instances::VSSGenerationTimetable::add_track_to_station(const std::string &name, const std::string &source,
                                                                  const std::string &target) {
    timetable.add_track_to_station(name, source, target, network);
}
