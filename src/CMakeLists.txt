add_library(
  ${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/include/Definitions.hpp
  ${PROJECT_SOURCE_DIR}/include/VSSModel.hpp
  ${PROJECT_SOURCE_DIR}/include/CustomExceptions.hpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/RailwayNetwork.hpp
  datastructure/RailwayNetwork.cpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/Train.hpp
  datastructure/Train.cpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/Timetable.hpp
  datastructure/Timetable.cpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/Station.hpp
  datastructure/Station.cpp
  ${PROJECT_SOURCE_DIR}/include/datastructure/Route.hpp
  datastructure/Route.cpp
  ${PROJECT_SOURCE_DIR}/include/probleminstances/VSSGenerationTimetable.hpp
  probleminstances/VSSGenerationTimetable.cpp
  probleminstances/SolVSSGenerationTimetable.cpp
  ${PROJECT_SOURCE_DIR}/include/MultiArray.hpp
  ${PROJECT_SOURCE_DIR}/include/solver/mip-based/VSSGenTimetableSolver.hpp
  solver/mip-based/VSSGenTimetableSolver_general.cpp
  solver/mip-based/VSSGenTimetableSolver_fixedRoutes.cpp
  solver/mip-based/VSSGenTimetableSolver_freeRoutes.cpp
  solver/mip-based/VSSGenTimetableSolver_helper.cpp)

# set include directories
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_BINARY_DIR}/include)

if(GUROBI_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${GUROBI_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} PUBLIC Gurobi::GurobiCXX)
endif()

# add tinyxml2
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/tinyxml2 extern/tinyxml2)

target_link_libraries(${PROJECT_NAME} PUBLIC project_options)

# add json
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/json extern/json)

target_link_libraries(${PROJECT_NAME} PUBLIC tinyxml2::tinyxml2)
target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json)

# add gsl
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/gsl extern/gsl)

target_link_libraries(${PROJECT_NAME} PUBLIC Microsoft.GSL::GSL)
