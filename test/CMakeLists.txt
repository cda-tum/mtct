if(NOT TARGET gtest OR NOT TARGET gmock)
  # Prevent overriding the parent project's compiler/linker settings on Windows
  set(gtest_force_shared_crt # cmake-lint: disable=C0103
      ON
      CACHE BOOL "" FORCE)
  add_subdirectory(${PROJECT_SOURCE_DIR}/extern/googletest "extern/googletest" EXCLUDE_FROM_ALL)
  set_target_properties(gtest gtest_main gmock gmock_main PROPERTIES FOLDER extern)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
package_add_test(
  ${PROJECT_NAME}_test
  "${PROJECT_NAME};${PROJECT_NAME}_cli_lib"
  ${CMAKE_CURRENT_SOURCE_DIR}/TestingWarning.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_gurobi.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_custom_exceptions.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_general_helper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_railwaynetwork.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_trainlist.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_station.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_eom.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_route.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_timetable.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_probleminstances.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_multiarray.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_gurobi_vss_gen.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_helper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_plog.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_general_performance_optimization_instances.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_gen_po_movingblock_mip_solver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_gurobi_vss_gen_using_mb_information.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_greedysimulator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_greedyheuristic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_mb_astar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_cli.cpp)

# Cheap smoke tests for the command line apps: printing the help text exercises every option definition, including the
# dependencies between them, and calling a solver app without arguments has to fail because of the required instance
# options.
foreach(app_name gen_po_moving_block_astar_testing gen_po_moving_block_mip_testing vss_generation_timetable_mip_testing
                 cli)
  add_test(NAME ${app_name}_help COMMAND ${PROJECT_NAME}_${app_name} --help)
endforeach()
foreach(app_name gen_po_moving_block_astar_testing gen_po_moving_block_mip_testing vss_generation_timetable_mip_testing)
  add_test(NAME ${app_name}_requires_instance COMMAND ${PROJECT_NAME}_${app_name})
  set_tests_properties(${app_name}_requires_instance PROPERTIES WILL_FAIL TRUE)
endforeach()

# The interactive app reads commands until it is told to stop, so it has to be checked that it actually does stop. The
# command behind the 'exit' would be echoed before it is run, so its name appearing in the output means the session
# continued; that check does not depend on what the test runner does with standard input. The timeout turns a session
# that never ends into a failing test rather than a run that never finishes.
set(CLI_EXIT_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/cli-exit.rail")
file(WRITE "${CLI_EXIT_SCRIPT}" "status\nexit\nnetwork new ReachedAfterExit\n")
add_test(NAME cli_exits COMMAND ${PROJECT_NAME}_cli --script "${CLI_EXIT_SCRIPT}")
set_tests_properties(cli_exits PROPERTIES TIMEOUT 60 FAIL_REGULAR_EXPRESSION "ReachedAfterExit")

# Without a script the app reads commands from standard input, which the test runner leaves empty, so the session has to
# end right away. Together with the debug flag this is the only way to reach the last part of main().
add_test(NAME cli_reads_standard_input COMMAND ${PROJECT_NAME}_cli --working-directory
                                               "${CMAKE_CURRENT_SOURCE_DIR}/data" --verbose)
set_tests_properties(cli_reads_standard_input PROPERTIES TIMEOUT 60)

# A script that does not exist is rejected before the session starts.
add_test(NAME cli_rejects_a_missing_script COMMAND ${PROJECT_NAME}_cli --script
                                                   "${CMAKE_CURRENT_BINARY_DIR}/no-such.rail")
set_tests_properties(cli_rejects_a_missing_script PROPERTIES WILL_FAIL TRUE TIMEOUT 60)
