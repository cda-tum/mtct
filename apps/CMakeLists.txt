# macro to add a executable with the given libraries
macro(ADD_SIM_EXECUTABLE appname)
  add_executable(${PROJECT_NAME}_${appname} ${CMAKE_CURRENT_SOURCE_DIR}/${appname}.cpp)
  target_link_libraries(${PROJECT_NAME}_${appname} PRIVATE ${PROJECT_NAME} ${ARGN})
endmacro()

add_sim_executable(vss_generation_timetable_mip_testing)
add_sim_executable(vss_generation_timetable_mip_inferred_testing)
add_sim_executable(vss_generation_timetable_mip_iterative_vss_testing)
