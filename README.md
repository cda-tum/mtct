# MTCT - Munich Train Control Toolkit
## CDA Rail - A tool for automated design of ETCS Hybrid Level 3 systems

Developers: Stefan Engels, Tom Peham, and Robert Wille

### Overview
The European Train Control System (ETCS) harmonizes many national train control systems.
Additionally, new specifications strive to increase the capacity of existing railway infrastructure.
ETCS Hybrid Level 3 (ETCS HL3) is of great practical interest to achieve shorter train following times.
It allows adding virtual subsections (VSS) that do not depend on trackside train detection (TTD) hardware.
However, finding optimal layouts is non-trivial and is currently done mainly manually.
In our research at the [Chair for Design Automation](https://www.cda.cit.tum.de/) of the [Technical University of Munich](https://www.tum.de/en/), we develop design methods to aid designers of such train control systems in automatically finding optimal layouts concerning various optimality criteria.

First attempts using satisfiability solvers [[1]](#references) and heuristics [[2]](#references) have been implemented at [https://github.com/cda-tum/da_etcs](https://github.com/cda-tum/da_etcs).
Since the methods used there cannot model continuous properties directly, simplifying assumptions were made.

This tool provides a flexible approach in which designers can individually trade off the efficiency of the solving process and the model's accuracy.
Currently, it supports an exact Mixed Integer Linear Programming (MILP) approach to generate minimal VSS layouts to satisfy a given timetable.
The tool is under active development, and more features will follow.

### Installation

#### System Requirements
The tool has been tested under Windows 11 (64-bit) using the MSVC compiler.
It should also be compatible with any current version of g++ supporting C++17 and a minimum CMake version of 3.19.

Moreover, the tool requires a local installation of a recent Gurobi [[3]](#references) version available at [https://www.gurobi.com/downloads/gurobi-software/](https://www.gurobi.com/downloads/gurobi-software/) as well as a valid [license](https://www.gurobi.com/solutions/licensing/).
For academic purposes, Gurobi offers [free academic licenses](https://www.gurobi.com/academia/academic-program-and-licenses/).
The project currently tests with Gurobi v10.0.1.

#### Build
To build the tool, go to the project folder and execute the following:

1) Configure CMake
    ```commandline
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    ```

2) Build the respective target.
    ```commandline
   cmake --build build --config Release
   ```

In some cases, CMake might not automatically find the Gurobi installation.
In that case you can manually set GUROBI_HOME in FindGurobi.cmake in the cmake directory, e.g., by adding
```
set(GUROBI_HOME "/home/opt/gurobi1001/linux64")
```
at the beginning.

### Usage

Currently, the tool provides only basic access via the command line. ```rail_vss_generation_timetable_mip_testing``` provides access to solving a specified instance at different levels of accuracy and with a predefined timeout.
It produces additional debugging output and saves the raw model and solution to a file.
The syntax is as follows
```commandline
.\build\apps\rail_vss_generation_timetable_mip_testing [model_name] [instance_path] [delta_t] [fix_routes] [discretize_vss_positions] [include_train_dynamics] [include_braking_curves] [use_pwl] [use_schedule_cuts] [timeout]
```
The parameters meaning is as follows:
- *delta_t*: Length of discretized time intervals in seconds.
- *fix_routes*: If true, the routes are fixed to the ones given in the instance. Otherwise, routing is part of the optimization.
- *discretize_vss_positions*: If true, the graphs edges are discretized in many short edges. VSS positions are then represented by vertices. If false, the VSS positions are encoded as continuous integers.
- *include_train_dynamics*: If true, the train dynamics (i.e., limited acceleration and deceleration) are included in the model.
- *include_braking_curves*: If true, the braking curves (i.e., the braking distance depending on the current speed has to be cleared) are included in the model.
- *use_pwl*: If true, the braking distances are approximated by piecewise linear functions with a fixed maximal error. Otherwise, they are modeled as quadratic functions and Gurobi's ability to solve these using spatial branching is used. Only relevant if include_braking_curves is true.
- *use_schedule_cuts*: If true, the formulation is strengthened using cuts implied by the schedule.
- *time_limit*: Time limit in seconds. No limit if negative.

Booleans have to be passed as numbers (0 = false or 1 = true).
Hence, the instance *SimpleStation* can be solved using default values by the following command:
```commandline
.\build\apps\rail_vss_generation_timetable_mip_testing SimpleStation .\test\example-networks\SimpleStation 15 1 0 1 1 0 1 -1
```

More command line functions will be added shortly.

Additionally, one can call the public methods to create, save, load, and solve respective instances in C++ directly.
For this, we refer to the source code's docstrings and example usages in the Google Tests found in the ```test``` folder.

Example networks can be found in ```test/example-networks/```.

## Contact Information
If you have any questions, feel free to contact us via etcs.cda@xcit.tum.de or by creating an issue on GitHub.

## References
[[1]](https://www.cda.cit.tum.de/files/eda/2021_date_automatic_design_verification_level3_etcs.pdf) Robert Wille and Tom Peham and Judith Przigoda and Nils Przigoda. **"Towards Automatic Design and Verification for Level 3 of the European Train Control System"**. Design, Automation and Test in Europe (DATE), 2021 ([pdf](https://www.cda.cit.tum.de/files/eda/2021_date_automatic_design_verification_level3_etcs.pdf))

[[2]](https://www.cda.cit.tum.de/files/eda/2022_rssrail_optimal_railway_routing_using_virtual_subsections.pdf) Tom Peham and Judith Przigoda and Nils Przigoda and Robert Wille. **"Optimal Railway Routing Using Virtual Subsections"**. Reliability, Safety and Security of Railway Systems (RSSRail), 2022 ([pdf](https://www.cda.cit.tum.de/files/eda/2022_rssrail_optimal_railway_routing_using_virtual_subsections.pdf))

[[3]](https://www.gurobi.com) Gurobi Optimization, LLC. **"Gurobi Optimizer Reference Manual"**. 2023