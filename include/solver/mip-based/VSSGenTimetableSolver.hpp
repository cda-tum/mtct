#pragma once
#include <string>
#include <filesystem>
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "gurobi_c++.h"
#include "unordered_map"
#include "MultiArray.hpp"


namespace cda_rail::solver::mip_based {
    class VSSGenTimetableSolver {
        private:
            cda_rail::instances::VSSGenerationTimetable instance;

            // Instance variables
            int dt, num_t, num_tr, num_edges, num_vertices, num_breakable_sections;
            std::vector<std::vector<int>> unbreakable_sections, no_border_vss_sections;
            std::vector<std::pair<int, int>> train_interval;
            std::vector<std::pair<int, int>> breakable_edges_pairs;
            std::vector<int> no_border_vss_vertices, relevant_edges, breakable_edges;
            bool fix_routes, discretize_vss_positions, include_train_dynamics, include_braking_curves, use_pwl, use_schedule_cuts;
            std::unordered_map<int, int> breakable_edge_indices;
            std::vector<std::pair<std::vector<int>, std::vector<int>>> fwd_bwd_sections;

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
            void create_brakelen_variables();

            // Constraint functions
            void create_general_constraints();
            void create_fixed_routes_constraints();
            void create_free_routes_constraints();
            void create_discretized_constraints();
            void create_non_discretized_constraints();
            void create_acceleration_constraints();
            void create_brakelen_constraints();


            // Helper functions for constraints
            void create_general_boundary_constraints();

            void create_general_schedule_constraints();
            void create_unbreakable_sections_constraints();
            void create_general_speed_constraints();
            void create_reverse_occupation_constraints();

            void create_fixed_routes_position_constraints();
            void create_boundary_fixed_routes_constraints();
            void create_fixed_routes_occupation_constraints();
            void create_fixed_route_schedule_constraints();
            void create_fixed_routes_impossibility_cuts();
            void create_fixed_routes_no_overlap_entry_exit_constraints();

            void create_non_discretized_general_constraints();
            void create_non_discretized_position_constraints();
            void create_non_discretized_free_route_constraints();
            void create_non_discretized_fixed_route_constraints();

            void create_free_routes_position_constraints();
            void create_free_routes_overlap_constraints();
            void create_boundary_free_routes_constraints();
            void create_free_routes_occupation_constraints();
            void create_free_routes_impossibility_cuts();
            void create_free_routes_no_overlap_entry_exit_constraints();

            // Objective
            void set_objective();

            // Helper functions
            std::vector<int> unbreakable_section_indices(int train_index) const;
            void calculate_fwd_bwd_sections();
            void calculate_fwd_bwd_sections_discretized();
            void calculate_fwd_bwd_sections_non_discretized();
            double get_max_brakelen(const int& tr) const;

            std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> common_entry_exit_vertices() const;

            struct TemporaryImpossibilityStruct {
                bool to_use;
                int t_before, t_after;
                double v_before, v_after;
                std::vector<int> edges_before, edges_after;
            };
            [[nodiscard]] TemporaryImpossibilityStruct get_temporary_impossibility_struct(const int& tr, const int& t) const;

            double max_distance_travelled(const int& tr, const int& time_steps, const double& v0, const double& a_max, const bool& braking_distance) const;

        public:
            // Constructors
            explicit VSSGenTimetableSolver(const cda_rail::instances::VSSGenerationTimetable& instance);
            explicit VSSGenTimetableSolver(const std::filesystem::path& instance_path);
            explicit VSSGenTimetableSolver(const std::string& instance_path);
            explicit VSSGenTimetableSolver(const char* instance_path);

            // Methods
            void solve(int delta_t = 15, bool fix_routes = true, bool discretize_vss_positions = false, bool include_train_dynamics = true, bool include_braking_curves = true, bool use_pwl = false, bool use_schedule_cuts = true, int time_limit = -1, bool debug = false, bool export_to_file = false, std::string file_name = "model");
    };
}