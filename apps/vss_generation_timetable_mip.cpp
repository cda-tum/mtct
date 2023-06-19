#include "solver/mip-based/VSSGenTimetableSolver.hpp"

int main(int argc, char** argv) {
    cda_rail::solver::mip_based::VSSGenTimetableSolver solver(argv[1]);

    solver.solve(15, true, false, true, false, false, true);
}