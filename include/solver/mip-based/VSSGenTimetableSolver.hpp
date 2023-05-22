#pragma once
#include <string>
#include <filesystem>
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "gurobi_c++.h"
#include "unordered_map"
#include "MultiArray.hpp"


namespace cda_rail::solver::mip_based {
    class VSSGenTimetableSolver {
        // TODO: Maybe add an abstract parent class later on
        private:
            cda_rail::instances::VSSGenerationTimetable instance;

            // Instance variables
            int dt, num_t, num_tr, num_edges, num_vertices;
            std::vector<std::vector<int>> unbreakable_sections, no_border_vss_sections;
            std::vector<std::pair<int, int>> train_interval;
            std::vector<int> no_border_vss_vertices;

            // Gurobi variables
            std::optional<GRBEnv> env;
            std::optional<GRBModel> model;
            std::unordered_map<std::string, MultiArray<GRBVar>> vars;

            // Helper functions
            void create_fixed_routes_variables();
            void create_general_variables();
            void create_vss_variables();

            void create_fixed_routes_train_movement_constraints();
            void create_boundary_fixed_routes_constraints();

            void set_objective();

            std::vector<int> unbreakable_section_indices(int train_index) const;

        public:
            // Constructors
            explicit VSSGenTimetableSolver(const cda_rail::instances::VSSGenerationTimetable& instance);
            explicit VSSGenTimetableSolver(const std::filesystem::path& instance_path);
            explicit VSSGenTimetableSolver(const std::string& instance_path);
            explicit VSSGenTimetableSolver(const char* instance_path);

            // Methods
            void solve(int delta_t = 15); // to be added with parameters and return type later
    };
}