#include "probleminstances/VSSGenerationTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "Definitions.hpp"

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

cda_rail::instances::VSSGenerationTimetable::VSSGenerationTimetable(const std::filesystem::path &p,
                                                                    bool every_train_must_have_route) {
    /**
     * Creates object and imports an instance from the given path, i.e.,
     * - the network from the folder "network"
     * - the timetable from the folder "timetable"
     * - the routes from the folder "routes"
     *
     * @param p the path to the folder where the instance should be imported from
     */

    this->network = cda_rail::Network::import_network(p / "network");
    this->timetable = cda_rail::Timetable::import_timetable(p / "timetable", this->network);
    this->routes = cda_rail::RouteMap::import_routes(p / "routes", this->network);
    if (!this->check_consistency(every_train_must_have_route)) {
        throw std::runtime_error("The imported instance is not consistent.");
    }
}

int cda_rail::instances::VSSGenerationTimetable::maxT() const {
    return timetable.maxT();
}

double cda_rail::instances::VSSGenerationTimetable::route_length(const std::string &train_name) const {
    return routes.length(train_name, network);
}
