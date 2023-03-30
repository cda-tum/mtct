add_library(
        ${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/include/Definitions.hpp
        ${PROJECT_SOURCE_DIR}/include/RailwayNetwork.hpp
        RailwayNetwork.cpp
)

# set include directories
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_BINARY_DIR}/include)

if(GUROBI_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC ${GUROBI_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC Gurobi::GurobiCXX)
endif()

target_link_libraries(${PROJECT_NAME} INTERFACE tinyxml2::tinyxml2)
target_link_libraries(${PROJECT_NAME} INTERFACE nlohmann::json)
