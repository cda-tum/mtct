#include "Timetable.hpp"

void cda_rail::Timetable::add_station(const std::string &name, const std::unordered_set<int> &tracks) {
    stations.push_back(cda_rail::Station{name, tracks});
    station_name_to_index[name] = stations.size() - 1;
}

void cda_rail::Timetable::add_station(const std::string &name) {
    stations.push_back(cda_rail::Station{name});
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

void cda_rail::Timetable::add_track_to_station(const std::string &name, int track, const cda_rail::Network &network) {
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

void cda_rail::Timetable::add_train(const std::string &name, int length, double max_speed, double acceleration,
                                    double deceleration, int t_0, double v_0, int entry, int t_n, double v_n, int exit,
                                    const cda_rail::Network &network) {
    if (!network.has_vertex(entry)) {
        throw std::out_of_range("Entry vertex does not exist.");
    }
    if (!network.has_vertex(exit)) {
        throw std::out_of_range("Exit vertex does not exist.");
    }
    if (has_train(name)) {
        throw std::out_of_range("Train already exists.");
    }
    trains.push_back(cda_rail::Train{name, length, max_speed, acceleration, deceleration});
    train_name_to_index[name] = trains.size() - 1;
    schedules.push_back(cda_rail::Schedule{t_0, v_0, entry, t_n, v_n, exit});
}

void cda_rail::Timetable::add_train(const std::string &name, int length, double max_speed, double acceleration,
                                    double deceleration, int t_0, double v_0, const std::string &entry, int t_n,
                                    double v_n, const std::string &exit, const cda_rail::Network &network) {
    if (!network.has_vertex(entry)) {
        throw std::out_of_range("Entry vertex does not exist.");
    }
    if (!network.has_vertex(exit)) {
        throw std::out_of_range("Exit vertex does not exist.");
    }
    if (has_train(name)) {
        throw std::out_of_range("Train already exists.");
    }
    add_train(name, length, max_speed, acceleration, deceleration, t_0, v_0, network.get_vertex_index(entry), t_n, v_n,
              network.get_vertex_index(exit), network);
}

void cda_rail::Timetable::add_stop(int train_index, int station_index, int begin, int end) {
    if (!has_train(train_index)) {
        throw std::out_of_range("Train does not exist.");
    }
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (begin < 0 || end < 0) {
        throw std::invalid_argument("Time cannot be negative.");
    }
    if (begin >= end) {
        throw std::invalid_argument("End time has to be after the start time.");
    }
    auto& stops_reference = schedules.at(train_index).stops;
    for (const auto& stop : stops_reference) {
        if (stop.station == station_index) {
            throw std::out_of_range("Train already stops at station.");
        }
        // Check if [begin, end] and [stop.begin, stop.end] overlap
        if (begin <= stop.end && end >= stop.begin) {
            throw std::invalid_argument("Train has another stop at this time.");
        }
    }

    stops_reference.push_back(cda_rail::ScheduledStop{begin, end, station_index});
}

void cda_rail::Timetable::add_stop(const std::string &train_name, int station_index, int begin, int end) {
    if (!has_train(train_name)) {
        throw std::out_of_range("Train does not exist.");
    }
    add_stop(train_name_to_index.at(train_name), station_index, begin, end);
}

void cda_rail::Timetable::add_stop(int train_index, const std::string &station_name, int begin, int end) {
    if (!has_station(station_name)) {
        throw std::out_of_range("Station does not exist.");
    }
    add_stop(train_index, station_name_to_index.at(station_name), begin, end);
}

void cda_rail::Timetable::add_stop(const std::string &train_name, const std::string &station_name, int begin, int end) {
    if (!has_train(train_name)) {
        throw std::out_of_range("Train does not exist.");
    }
    if (!has_station(station_name)) {
        throw std::out_of_range("Station does not exist.");
    }
    add_stop(train_name_to_index.at(train_name), station_name_to_index.at(station_name), begin, end);
}
