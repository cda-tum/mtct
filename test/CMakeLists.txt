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
  ${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/test_railwaynetwork.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_probleminstances.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_multiarray.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_gurobi_vss_gen.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_helper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_plog.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_abstract_data_structure.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_general_performance_optimization_instances.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_gen_po_movingblock_mip_solver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_gurobi_vss_gen_using_mb_information.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_greedysimulator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_greedyheuristic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_mb_astar.cpp)
