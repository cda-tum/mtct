# macro to add a executable with the given libraries
macro(ADD_SIM_EXECUTABLE appname)
  add_executable(${PROJECT_NAME}_${appname} ${CMAKE_CURRENT_SOURCE_DIR}/${appname}.cpp)
  target_link_libraries(${PROJECT_NAME}_${appname} PRIVATE ${PROJECT_NAME} ${ARGN})
endmacro()

add_sim_executable(vss_generation_timetable_mip_testing)
add_sim_executable(vss_generation_timetable_mip_iterative_vss_testing)
add_sim_executable(vss_generation_timetable_iterative_parameter_testing)
add_sim_executable(gen_po_moving_block_lazy_vss_gen_testing)
add_sim_executable(gen_po_moving_block_lazy_testing)
add_sim_executable(gen_po_moving_block_simplified_vss_gen_testing)
