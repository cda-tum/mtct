![OS](https://img.shields.io/badge/os-linux%20%7C%20macos%20%7C%20windows-blue?style=flat-square)
[![C++](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/cpp-ci.yml?label=c%2B%2B&logo=github&style=flat-square&branch=main)](https://github.com/cda-tum/mtct/actions/workflows/cpp-ci.yml)
[![CodeQL](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/codeql-analysis.yml?label=CodeQL&logo=github&style=flat-square&branch=main)](https://github.com/cda-tum/mtct/actions/workflows/codeql-analysis.yml)
[![codecov](https://img.shields.io/codecov/c/github/cda-tum/mtct?label=Coverage&logo=codecov&style=flat-square&branch=main)](https://codecov.io/gh/cda-tum/mtct)
[![License](https://img.shields.io/github/license/cda-tum/mtct?label=License&style=flat-square&branch=main)](https://github.com/cda-tum/mtct/blob/main/LICENSE)

# MTCT - Munich Train Control Toolkit

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="_img/logo-train-control-toolkit-white-medium.png" width="60%">
    <img src="_img/logo-train-control-toolkit-medium.png" width="60%">
  </picture>
</p>

## A Tool for Automated Design of ETCS Systems with Hybrid Train Detection (formerly ETCS Hybrid Level 3)

Developers: Stefan Engels, Tom Peham, and Robert Wille

### Overview

The European Train Control System (ETCS) harmonizes many national train control systems.
Additionally, new specifications strive to increase the capacity of existing railway infrastructure.
ETCS Hybrid Level 3 (ETCS HL3) is of great practical interest to achieve shorter train following times.
It allows adding virtual subsections (VSS) that do not depend on trackside train detection (TTD) hardware.
However, finding optimal layouts is non-trivial and is currently done mainly manually.
In our research at the [Chair for Design Automation](https://www.cda.cit.tum.de/) of the [Technical University of Munich](https://www.tum.de/en/), we develop design methods to aid designers of such train control systems in automatically finding optimal layouts concerning various optimality criteria.
A preprint describing the arising design tasks in more detail is available [[6]](#references)

First attempts using satisfiability solvers [[1]](#references) and heuristics [[2]](#references) have been implemented at [https://github.com/cda-tum/da_etcs](https://github.com/cda-tum/da_etcs).
Since the methods used there cannot model continuous properties directly, simplifying assumptions were made.

This tool provides a flexible approach in which designers can individually trade off the efficiency of the solving process and the model's accuracy.
Currently, it supports an exact Mixed Integer Linear Programming (MILP) approach to generate minimal VSS layouts to satisfy a given timetable [[3]](#references).
The runtime has been improved using an iterative approach [[4]](#references) and the usage of precomputed routing information.
Finally, optimal train routings can also be computed in a moving block controlled environment [[5]](#references).
The tool is under active development, and more features will follow.

### Installation

#### System Requirements

The tool has been tested under Windows 11 (64-bit) using the MSVC compiler.
It should also be compatible with any current version of g++ supporting C++17 and a minimum CMake version of 3.19.

Moreover, the tool requires a local installation of a recent Gurobi [[7]](#references) version available at [https://www.gurobi.com/downloads/gurobi-software/](https://www.gurobi.com/downloads/gurobi-software/) as well as a valid [license](https://www.gurobi.com/solutions/licensing/).
For academic purposes, Gurobi offers [free academic licenses](https://www.gurobi.com/academia/academic-program-and-licenses/).
The project currently tests with Gurobi v11.0.3.

#### Build

To build the tool, go to the project folder and execute the following:

1. Configure CMake

   ```commandline
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   ```

2. Build the respective target.
   ```commandline
   cmake --build build --config Release
   ```

When compiling, CMake automatically searches for Gurobi at the default locations, i.e.,

- `C:/gurobi<VERSION>/win64` for Windows systems
- `/home/opt/gurobi<VERSION>/linux64` for Linux systems
- `/Library/gurobi<VERSION>/macos_universal2` for MacOS systems
  where `<VERSION>` denotes the installed Gurobi version.

If this does not work, please set the OS environment variable GUROBI_HOME to the respective install directory.
This way, CMake can find Gurobi even in non-standard directories.

If you are using Windows, make sure that Gurobi's `bin` folder, i.e., `<installdir>\bin`, is appended to the `Path` environmental variable.
The above variables are usually automatically set, if Gurobi is installed using the installer with administrator privileges.
Otherwise, they have to be set manually, see also https://support.gurobi.com/hc/en-us/articles/360060996432-How-do-I-install-Gurobi-on-Windows-without-administrator-credentials-

### Usage

Currently, the tool provides only basic access via the command line and supports the generation of minimal VSS layouts. More command line functions will be added shortly. Example networks can be found in `test/example-networks/`.

#### MILP Based Routing under Moving Block

`rail_gen_po_moving_block_lazy_testing` and `rail_gen_po_moving_block_lazy_vss_gen_testing` are command line interfaces for optimal train routing using moving block train control. Various settings can be set by the user affecting the solving process. More precisely, different strategies on how to select lazy constraints can be used.
The syntax is as follows

```commandline
.\build\apps\rail_gen_po_moving_block_lazy_testing [model_name] [instance_path] [use_lazy] [reverse_headways] [higher_velocities] [lazy_strategy] [train_strategy] [timeout]
.\build\apps\rail_gen_po_moving_block_lazy_vss_gen_testing [model_name] [instance_path] [use_lazy] [reverse_headways] [higher_velocities] [lazy_strategy] [train_strategy] [timeout]
```

The parameters are defined as follows:

- _use_lazy_: If true, headway constraints are generated on demand as lazy constraints. If false, the full model is explicitly specified before solving.
- _reverse_headways_: If a violated headway constraint is added, then also constraints are added, in which the train order is reversed.
- _higher_velocities_: If a violated headway constraint is added, it is also enforced for higher velocities than the one deduced from the current solution.
- _lazy_strategy_: If 0, only violated headway constraints are added. If 1, then the callback of adding lazy constraints stops after it has found a single violated constraint. If 2, all constraints that are checked for violations are added whether or not they are violated by the current solution.
- _train_strategy_: If 0, only trains directly following each other are checked for violations. If 1, also constraints for trains where a third train might run in between are added.
- _timeout_: Time limit in seconds. No limit if negative.

If the instance contains a general timetable, i.e., times are specified with lower and upper bounds only, then `rail_gen_po_moving_block_lazy_testing` should be used. Otherwise, i.e., if the timetable is specified with exact arrival and departure times, then `rail_gen_po_moving_block_lazy_vss_gen_testing` should be used.

Booleans have to be passed as numbers (0 = false or 1 = true).
Hence, the instance _SimpleStation_ could be solved by the following command:

```commandline
.\build\apps\rail_gen_po_moving_block_lazy_vss_gen_testing SimpleStation .\test\example-networks\SimpleStation 1 0 0 0 0 -1
```

A further variant with simpler headway constraints has also been implemented. In this case, the train separation might not be accurate but the solving time might improve significantly. This might be worth considering if the results are only used to make preliminary decisions. The respective command line apps are `rail_gen_po_moving_block_simplified_testing` and `ail_gen_po_moving_block_simplified_vss_gen_testing` respectively with the following parameters

- _use_simplified_headways_: If true, the simplified headways are used.
- _strengthen_vertex_headway_constraints_: If true, simple linear constraints are used to improve headway estimation at vertices based on the velocity variables.
- _use_lazy_: If true, headway constraints are generated on demand as lazy constraints. If false, the full model is explicitly specified before solving.
- _lazy_strategy_: If 0, only violated headway constraints are added. If 1, then the callback of adding lazy constraints stops after it has found a single violated constraint. If 2, all constraints that are checked for violations are added whether or not they are violated by the current solution.
- _train_strategy_: If 0, only trains directly following each other are checked for violations. If 1, also constraints for trains where a third train might run in between are added.
- _timeout_: Time limit in seconds. No limit if negative.

This functionality is based on [[5]](#references).

#### MILP Based VSS Generation

`rail_vss_generation_timetable_mip_testing` provides access to generating minimal VSS layouts given a specific timetable at different levels of accuracy and with a predefined timeout.
It produces additional debugging output and saves the raw model and solution to a file.
The syntax is as follows

```commandline
.\build\apps\rail_vss_generation_timetable_mip_testing [model_name] [instance_path] [delta_t] [fix_routes] [discretize_vss_positions] [include_train_dynamics] [include_braking_curves] [use_pwl] [use_schedule_cuts] [timeout]
```

The parameters meaning is as follows:

- _delta_t_: Length of discretized time intervals in seconds.
- _fix_routes_: If true, the routes are fixed to the ones given in the instance. Otherwise, routing is part of the optimization.
- _discretize_vss_positions_: If true, the graphs edges are discretized in many short edges. VSS positions are then represented by vertices. If false, the VSS positions are encoded as continuous integers.
- _include_train_dynamics_: If true, the train dynamics (i.e., limited acceleration and deceleration) are included in the model.
- _include_braking_curves_: If true, the braking curves (i.e., the braking distance depending on the current speed has to be cleared) are included in the model.
- _use_pwl_: If true, the braking distances are approximated by piecewise linear functions with a fixed maximal error. Otherwise, they are modeled as quadratic functions and Gurobi's ability to solve these using spatial branching is used. Only relevant if include_braking_curves is true.
- _use_schedule_cuts_: If true, the formulation is strengthened using cuts implied by the schedule.
- _time_limit_: Time limit in seconds. No limit if negative.

Booleans have to be passed as numbers (0 = false or 1 = true).
Hence, the instance _SimpleStation_ can be solved using default values by the following command:

```commandline
.\build\apps\rail_vss_generation_timetable_mip_testing SimpleStation .\test\example-networks\SimpleStation 15 1 0 1 1 0 1 -1
```

This functionality is based on [[3]](#references).

#### Iterative Approach

An iterative approach has been implemented, which can significantly improve the runtime. It uses the continuous model for placing VSS borders. The syntax is as follows

```commandline
.\build\apps\rail_vss_generation_timetable_mip_iterative_vss_testing [model_name] [instance_path] [delta_t] [fix_routes] [include_train_dynamics] [include_braking_curves] [use_pwl] [use_schedule_cuts] [iterate_vss] [optimality_strategy] [timeout] [output_path - optional]
```

The parameters meaning is as follows:

- _delta_t_: Length of discretized time intervals in seconds.
- _fix_routes_: If true, the routes are fixed to the ones given in the instance. Otherwise, routing is part of the optimization.
- _include_train_dynamics_: If true, the train dynamics (i.e., limited acceleration and deceleration) are included in the model.
- _include_braking_curves_: If true, the braking curves (i.e., the braking distance depending on the current speed has to be cleared) are included in the model.
- _use_pwl_: If true, the braking distances are approximated by piecewise linear functions with a fixed maximal error. Otherwise, they are modeled as quadratic functions and Gurobi's ability to solve these using spatial branching is used. Only relevant if include_braking_curves is true.
- _use_schedule_cuts_: If true, the formulation is strengthened using cuts implied by the schedule.
- _iterate_vss_: If true, the solver proceeds iteratively, i.e., it will start by trying to solve a restricted model, which is easier to solve, and only slowly increases its size. In many cases, already on such restricted models the optimal solution can be found.
- _optimality_strategy_: 0 (Optimal): The proven optimal solution is found; 1 (TradeOff): The restricted model is solved to optimality. The solution is returned even if it is not proven to be globally optimal. Experiments show that it is likely optimal, but the algorithm provides no guarantee; 2 (Feasible): The algorithm focuses only on finding a (probably good) feasible solution. It is likely not the optimal solution, but only close to optimal. No guarantee is provided.
- _time_limit_: Time limit in seconds. No limit if negative.
- _output_path_: The path in which the solution is written. The default is the current working directory.

Booleans have to be passed as numbers (0 = false or 1 = true).
Hence, the instance _SimpleStation_ can be solved using default values by the following command:

```commandline
.\build\apps\rail_vss_generation_timetable_mip_iterative_vss_testing SimpleStation .\test\example-networks\SimpleStation 15 1 1 1 0 1 1 0 -1
```

This functionality is based on [[4]](#references).

#### Using Precomputed Solution Information

If a [routing under moving block](#milp-based-routing-under-moving-block) has been precomputed, this information can be used when generating minimal VSS layouts. For this, the app `rail_vss_generation_timetable_using_mb_information_testing` can be used.
The syntax is as follows

```commandline
.\build\apps\rail_vss_generation_timetable_using_mb_information_testing [model_name] [instance_path] [delta_t] [use_mb_information] [fix_stop_positions] [fix_exact_positions] [fix_exact_velocities] [hint_approximate_positions] [fix_order_on_edges] [usw_pwl] [output_path - optional]
```

where the parameters are

- _delta_t_: Length of discretized time intervals in seconds.
- _use_mb_information_: If true, information from a moving block routing is used. Most importantly, the specific edges used by each train are fixed.
- _fix_stop_positions_: If true, the approximate stopping position in each station is fixed.
- _fix_exact_positions_: If true, the position at each time step is bound with additional constraints based on the timing of the moving block solution.
- _fix_exact_velocities_: If true, the velocity at each time step is bound with additional constraints based on the train speed of the moving block solution.
- _hint_approximate_positions_: If true, variable hints are passed to Gurobi.
- _fix_order_on_edges_: If true, on each edge the order in which trains traverse is fixed.
- _use_pwl_: If true, the braking distances are approximated by piecewise linear functions with a fixed maximal error. Otherwise, they are modeled as quadratic functions and Gurobi's ability to solve these using spatial branching is used. Only relevant if include_braking_curves is true.
- _time_limit_: Time limit in seconds. No limit if negative.

Booleans have to be passed as numbers (0 = false or 1 = true).

This functionality is based on unpublished work.

#### Access via C++

Additionally, one can call the public methods to create, save, load, and solve respective instances in C++ directly.
For this, we refer to the source code's docstrings and example usages in the Google Tests found in the `test` folder.

## Contact Information

If you have any questions, feel free to contact us via etcs.cda@xcit.tum.de or by creating an issue on GitHub.

## References

[[1]](https://www.cda.cit.tum.de/files/eda/2021_date_automatic_design_verification_level3_etcs.pdf) Robert Wille and Tom Peham and Judith Przigoda and Nils Przigoda. **"Towards Automatic Design and Verification for Level 3 of the European Train Control System"**. Design, Automation and Test in Europe (DATE), 2021 ([doi](https://doi.org/10.23919/DATE51398.2021.9473935), [pdf](https://www.cda.cit.tum.de/files/eda/2021_date_automatic_design_verification_level3_etcs.pdf))

[[2]](https://www.cda.cit.tum.de/files/eda/2022_rssrail_optimal_railway_routing_using_virtual_subsections.pdf) Tom Peham and Judith Przigoda and Nils Przigoda and Robert Wille. **"Optimal Railway Routing Using Virtual Subsections"**. Reliability, Safety and Security of Railway Systems (RSSRail), 2022 ([doi](https://doi.org/10.1007/978-3-031-05814-1_5), [pdf](https://www.cda.cit.tum.de/files/eda/2022_rssrail_optimal_railway_routing_using_virtual_subsections.pdf))

[[3]](https://drops.dagstuhl.de/opus/volltexte/2023/18767/pdf/OASIcs-ATMOS-2023-6.pdf) Stefan Engels and Tom Peham and Robert Wille. **"A Symbolic Design Method for ETCS Hybrid Level 3 at Different Degrees of Accuracy"**. Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS), 2023 ([doi](https://doi.org/10.4230/OASIcs.ATMOS.2023.6), [pdf](https://drops.dagstuhl.de/opus/volltexte/2023/18767/pdf/OASIcs-ATMOS-2023-6.pdf))

[[4]](https://www.cda.cit.tum.de/files/eda/2024_date_lbr_iterative_da_for_train_control_with_htd.pdf) Stefan Engels and Robert Wille. **"Late Breaking Results: Iterative Design Automation for Train Control with Hybrid Train Detection"**. Design, Automation and Test in Europe (DATE), 2024 ([IEEE Xplore](https://ieeexplore.ieee.org/document/10546590), [pdf](https://www.cda.cit.tum.de/files/eda/2024_date_lbr_iterative_da_for_train_control_with_htd.pdf))

[[5]](https://www.cda.cit.tum.de/files/eda/2024_fedcsis_lazy_constraint_selection_strategies_moving_block.pdf) Stefan Engels and Robert Wille. **"Comparing Lazy Constraint Selection Strategies in Train Routing with Moving Block Control"**. Conference on Computer Science and Intelligence Systems (FedCSIS), 2024 ([arXiv](https://arxiv.org/abs/2405.18977), [pdf](https://www.cda.cit.tum.de/files/eda/2024_fedcsis_lazy_constraint_selection_strategies_moving_block.pdf))

[[6]](https://www.cda.cit.tum.de/files/eda/2024_arXiv_etcs_design_tasks_and_complexity.pdf) Stefan Engels and Tom Peham and Judith Przigoda and Nils Przigoda and Robert Wille. **"Design Tasks and Their Complexity for the European Train Control System with Hybrid Train Detection"**. Preprint, 2024 ([arXiv](https://arxiv.org/abs/2308.02572), [pdf](https://www.cda.cit.tum.de/files/eda/2024_arXiv_etcs_design_tasks_and_complexity.pdf))

[[7]](https://www.gurobi.com) Gurobi Optimization, LLC. **"Gurobi Optimizer Reference Manual"**. 2024
