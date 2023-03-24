#include "Timetable.hpp"

void cda_rail::Timetable::add_station(const std::string &name, std::unordered_set<int> &tracks) {
    stations.emplace_back(name, tracks);
    station_name_to_index[name] = stations.size() - 1;
}

void cda_rail::Timetable::add_station(const std::string &name) {
    stations.emplace_back(name);
    station_name_to_index[name] = stations.size() - 1;
}

int cda_rail::Timetable::get_station_index(const std::string &name) const {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    return station_name_to_index.at(name);
}

const cda_rail::Station &cda_rail::Timetable::get_station(int index) const {
    if (!has_station(index)) {
        throw std::out_of_range("Station does not exist.");
    }
    return stations.at(index);
}

const cda_rail::Station &cda_rail::Timetable::get_station(const std::string &name) const {
    return get_station(get_station_index(name));
}

bool cda_rail::Timetable::has_station(const std::string &name) const {
    return station_name_to_index.find(name) != station_name_to_index.end();
}

bool cda_rail::Timetable::has_train(const std::string &name) const {
    return train_name_to_index.find(name) != train_name_to_index.end();
}

bool cda_rail::Timetable::has_train(int index) const {
    return (index >= 0 && index < trains.size());
}

bool cda_rail::Timetable::has_station(int index) const {
    return (index >= 0 && index < stations.size());
}

int cda_rail::Timetable::get_train_index(const std::string &name) const {
    if (!has_train(name)) {
        throw std::out_of_range("Train does not exist.");
    }
    return train_name_to_index.at(name);
}

const cda_rail::Train &cda_rail::Timetable::get_train(int index) const {
    if (!has_train(index)) {
        throw std::out_of_range("Train does not exist.");
    }
    return trains.at(index);
}

const cda_rail::Train &cda_rail::Timetable::get_train(const std::string &name) const {
    if (!has_train(name)) {
        throw std::out_of_range("Train does not exist.");
    }
    return get_train(get_train_index(name));
}

const cda_rail::Schedule &cda_rail::Timetable::get_schedule(int index) const {
    if (!has_train(index)) {
        throw std::out_of_range("Train does not exist.");
    }
    return schedules.at(index);
}

const cda_rail::Schedule &cda_rail::Timetable::get_schedule(const std::string &train_name) const {
    if (!has_train(train_name)) {
        throw std::out_of_range("Train does not exist.");
    }
    return get_schedule(get_train_index(train_name));
}

void cda_rail::Timetable::add_track_to_station(int station_index, int track, const cda_rail::Network &network) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(track)) {
        throw std::out_of_range("Track does not exist.");
    }
    stations.at(station_index).tracks.insert(track);
}

void cda_rail::Timetable::add_track_to_station(const td::string &name, int track, const cda_rail::Network &network) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(track)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(get_station_index(name), track, network);
}

void
cda_rail::Timetable::add_track_to_station(int station_index, int source, int target, const cda_rail::Network &network) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(station_index, network.get_edge_index(source, target), network);
}

void
cda_rail::Timetable::add_track_to_station(const std::string &name, int source, int target, const cda_rail::Network &network) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(get_station_index(name), network.get_edge_index(source, target), network);
}

void cda_rail::Timetable::add_track_to_station(int station_index, const std::string &source, const std::string &target,
                                               const cda_rail::Network &network) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(station_index, network.get_edge_index(source, target), network);
}
