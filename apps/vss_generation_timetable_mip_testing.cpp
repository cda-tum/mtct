#include "solver/mip-based/VSSGenTimetableSolver.hpp"

int main(int argc, char** argv) {

    if (argc != 11) {
        std::cout << "Expected 10 arguments, got " << argc-1 << std::endl;
        std::exit(-1);
    }

    std::string model_name = argv[1];
    std::string instance_path = argv[2];
    cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

    std::cout << "Instance " << model_name << " loaded at " << instance_path << std::endl;

    int delta_t = std::stoi(argv[3]);
    bool fix_routes = std::stoi(argv[4]);
    bool discretize = std::stoi(argv[5]);
    bool include_acceleration_deceleration = std::stoi(argv[6]);
    bool include_breaking_distance = std::stoi(argv[7]);
    bool use_pwl = std::stoi(argv[8]);
    bool use_cuts = std::stoi(argv[9]);
    int timeout = std::stoi(argv[10]);

    std::cout << "The following parameters were passed to the toolkit:" << std::endl;
    std::cout << "   delta_t: " << delta_t << std::endl;
    if (fix_routes) {
        std::cout << "   routes are fixed" << std::endl;
    }
    if (discretize) {
        std::cout << "   ´the graph is preprocessed" << std::endl;
    }
    if (include_acceleration_deceleration) {
        std::cout << "   acceleration and deceleration are included" << std::endl;
    }
    if (include_breaking_distance) {
        std::cout << "   breaking distance is included" << std::endl;
    }
    if (use_pwl) {
        std::cout << "   piecewise linear functions are used" << std::endl;
    }
    if (use_cuts) {
        std::cout << "   cuts are used" << std::endl;
    }
    std::cout << "   timeout: " << timeout << "s" << std::endl;

    solver.solve(delta_t, fix_routes, discretize, include_acceleration_deceleration, include_breaking_distance, use_pwl, use_cuts, timeout);
}