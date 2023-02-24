add_library(
        ${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/include/Definitions.hpp
        ${PROJECT_SOURCE_DIR}/include/RailwayNetwork.hpp
        RailwayNetwork.cpp
)

# set include directories
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_BINARY_DIR}/include)

include_directories(C:/gurobi1001/win64/include)
target_link_libraries(${PROJECT_NAME} C:/gurobi1001/win64/lib/gurobi100.lib)
