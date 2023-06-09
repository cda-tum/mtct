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
            int dt, num_t, num_tr, num_edges, num_vertices, num_breakable_sections;
            std::vector<std::vector<int>> unbreakable_sections, no_border_vss_sections;
            std::vector<std::pair<int, int>> train_interval;
            std::vector<std::pair<int, int>> breakable_edges_pairs;
            std::vector<int> no_border_vss_vertices, relevant_edges, breakable_edges;
            bool fix_routes, discretize, include_acceleration_deceleration, include_breaking_distances;
            std::unordered_map<int, int> breakable_edge_indices;

            // Gurobi variables
            std::optional<GRBEnv> env;
            std::optional<GRBModel> model;
            std::unordered_map<std::string, MultiArray<GRBVar>> vars;

            // Variable functions
            void create_general_variables();
            void create_fixed_routes_variables();
            void create_free_routes_variables();
            void create_discretized_variables();
            void create_non_discretized_variables();
            void create_breaklen_variables();

            // Constraint functions
            void create_general_constraints();
            void create_fixed_routes_constraints();
            void create_free_routes_constraints();
            void create_discretized_constraints();
            void create_non_discretized_constraints();
            void create_acceleration_constraints();
            void create_breaklen_constraints();


            // Helper functions for constraints
            void create_general_schedule_constraints();
            void create_unbreakable_sections_constraints();
            void create_general_speed_constraints();
            void create_reverse_occupation_constraints();

            void create_fixed_routes_position_constraints();
            void create_boundary_fixed_routes_constraints();
            void create_fixed_routes_occupation_constraints();
            void create_fixed_route_schedule_constraints();

            void create_non_discretized_general_constraints();
            void create_non_discretized_position_constraints();
            void create_non_discretized_free_route_constraints();
            void create_non_discretized_fixed_route_constraints();

            // Objective
            void set_objective();

            // Helper functions
            std::vector<int> unbreakable_section_indices(int train_index) const;

        public:
            // Constructors
            explicit VSSGenTimetableSolver(const cda_rail::instances::VSSGenerationTimetable& instance);
            explicit VSSGenTimetableSolver(const std::filesystem::path& instance_path);
            explicit VSSGenTimetableSolver(const std::string& instance_path);
            explicit VSSGenTimetableSolver(const char* instance_path);

            // Methods
            void solve(int delta_t = 15, bool fix_routes = true, bool discretize = false, bool include_acceleration_deceleration = true, bool include_breaking_distances = true); // to be added with parameters and return type later
    };
}