add_library(
        ${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/include/Definitions.hpp
        ${PROJECT_SOURCE_DIR}/include/RailwayNetwork.hpp
        RailwayNetwork.cpp
        ${PROJECT_SOURCE_DIR}/include/parsing/tinyxml2.h
        ${PROJECT_SOURCE_DIR}/include/parsing/json.hpp
        parsing/tinyxml2.cpp
        ${PROJECT_SOURCE_DIR}/include/Train.hpp
        Train.cpp
        ${PROJECT_SOURCE_DIR}/include/Timetable.hpp
        Timetable.cpp
)

# set include directories
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_BINARY_DIR}/include)

if(GUROBI_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC ${GUROBI_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC Gurobi::GurobiCXX)
endif()
