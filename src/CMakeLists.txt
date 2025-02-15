add_library(
  ${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/include/EOMHelper.hpp
  EOMHelper.cpp
  ${PROJECT_SOURCE_DIR}/include/Definitions.hpp
  ${PROJECT_SOURCE_DIR}/include/VSSModel.hpp
  ${PROJECT_SOURCE_DIR}/include/CustomExceptions.hpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/RailwayNetwork.hpp
  datastructure/RailwayNetwork.cpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/Train.hpp
  datastructure/Train.cpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/Timetable.hpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/GeneralTimetable.hpp
  datastructure/Timetable.cpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/Station.hpp
  datastructure/Station.cpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/Route.hpp
  datastructure/Route.cpp
  ${PROJECT_SOURCE_DIR}/include/probleminstances/GeneralProblemInstance.hpp
  ${PROJECT_SOURCE_DIR}/include/probleminstances/GeneralPerformanceOptimizationInstance.hpp
  ${PROJECT_SOURCE_DIR}/include/probleminstances/VSSGenerationTimetable.hpp
  probleminstances/VSSGenerationTimetable.cpp
  probleminstances/SolVSSGenerationTimetable.cpp
  probleminstances/GeneralPerformanceOptimizationInstance.cpp
  ${PROJECT_SOURCE_DIR}/include/MultiArray.hpp
  ${PROJECT_SOURCE_DIR}/include/solver/mip-based/VSSGenTimetableSolver.hpp
  ${PROJECT_SOURCE_DIR}/include/solver/GeneralSolver.hpp
  ${PROJECT_SOURCE_DIR}/include/solver/mip-based/GeneralMIPSolver.hpp
  ${PROJECT_SOURCE_DIR}/include/solver/mip-based/GenPOMovingBlockMIPSolver.hpp
  solver/mip-based/VSSGenTimetableSolver_general.cpp
  solver/mip-based/VSSGenTimetableSolver_fixedRoutes.cpp
  solver/mip-based/VSSGenTimetableSolver_freeRoutes.cpp
  solver/mip-based/VSSGenTimetableSolver_helper.cpp
  solver/mip-based/VSSGenTimetableSolver_MovingBlockInformation.cpp
  solver/mip-based/GenPOMovingBlockMIPSolver.cpp
  solver/mip-based/GenPOMovingBlockMIPSolver_SolutionExtraction.cpp
  solver/mip-based/GenPOMovingBlockMIPSolver_Lazy.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/SimulationInstance.hpp
  simulation/SimulationInstance.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/RoutingSolution.hpp
  simulation/RoutingSolution.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/RoutingSolutionSet.hpp
  simulation/RoutingSolutionSet.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/RoutingSolver.hpp
  simulation/RoutingSolver.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/SolverResult.hpp
  simulation/SolverResult.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/EdgeTrajectory.hpp
  simulation/EdgeTrajectory.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/TrainTrajectory.hpp
  simulation/TrainTrajectory.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/TrainTrajectorySet.hpp
  simulation/TrainTrajectorySet.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/SpeedTargets.hpp
  simulation/SpeedTargets.cpp
  ${PROJECT_SOURCE_DIR}/include/simulation/Objectives.hpp
  simulation/Objectives.cpp
  ${PROJECT_SOURCE_DIR}/extern/openGA/src/openGA.hpp)

# set include directories
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_BINARY_DIR}/include)
# target_include_directories(${PROJECT_NAME} PUBLIC ${gtest_SOURCE_DIR}/include) # for FRIEND_TEST usability

if(GUROBI_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${GUROBI_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} PUBLIC Gurobi::GurobiCXX)
endif()

# add tinyxml2
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/tinyxml2 extern/tinyxml2)
target_link_libraries(${PROJECT_NAME} PUBLIC project_options)
target_link_libraries(${PROJECT_NAME} PUBLIC tinyxml2::tinyxml2)

# add json
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/json extern/json)
target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json)

# add gsl
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/gsl extern/gsl)
target_link_libraries(${PROJECT_NAME} PUBLIC Microsoft.GSL::GSL)

# add plog
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/plog extern/plog)
target_link_libraries(${PROJECT_NAME} PUBLIC plog::plog)

# add gtest for FRIEND_TEST
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/googletest extern/googletest)
target_link_libraries(${PROJECT_NAME} PUBLIC gtest)
