#include "solver/mip-based/VSSGenTimetableSolver.hpp"
#include "gurobi_c++.h"
#include "MultiArray.hpp"

std::vector<int>
cda_rail::solver::mip_based::VSSGenTimetableSolver::unbreakable_section_indices(int train_index) const {
    /**
     * This function returns the indices of the unbreakable sections that are traversed by the train with index train_index
     * @param train_index index of the train
     * @return vector of indices
     */

    std::vector<int> indices;
    const auto& tr_name = instance.get_train_list().get_train(train_index).name;
    const auto& tr_route = instance.get_route(tr_name).get_edges();
    for (int i = 0; i < unbreakable_sections.size(); ++i) {
        bool edge_found = false;
        // If unbreakable_section[i] (of type vector) and tr_route (of type vector) overlap (have a common element), add i to indices
        for (int j0 = 0; j0 < unbreakable_sections[i].size() && !edge_found; ++j0) {
            for (int j1 = 0; j1 < tr_route.size() && !edge_found; ++j1) {
                if (unbreakable_sections[i][j0] == tr_route[j1]) {
                    indices.push_back(i);
                    edge_found = true;
                }
            }
        }
    }

    return indices;
}