# macro to add a test executable for one of the project libraries
macro(PACKAGE_ADD_TEST testname linklibs)
  # create an executable in which the tests will be stored
  add_executable(${testname} ${ARGN})
  # link the Google test infrastructure and a default main function to the test executable.
  target_link_libraries(${testname} PRIVATE ${linklibs} gmock gtest_main)
  file(TO_CMAKE_PATH "${CMAKE_CURRENT_BINARY_DIR}" warning_root_dir)
  target_compile_definitions(${testname} PRIVATE CDA_RAIL_TEST_WARNING_ROOT="${warning_root_dir}")

  set(warning_file "${CMAKE_CURRENT_BINARY_DIR}/${testname}_warnings.log")
  file(REMOVE "${warning_file}")

  # discover tests
  gtest_discover_tests(
    ${testname}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DISCOVERY_TIMEOUT 60
    TEST_LIST ${testname}_test_list
    PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

  set(warning_config_file "${CMAKE_CURRENT_BINARY_DIR}/${testname}_warning_config.cmake")
  file(WRITE "${warning_config_file}" "foreach(current_test IN LISTS ${testname}_test_list)\n")
  file(APPEND "${warning_config_file}" "  set_tests_properties(\"\${current_test}\" PROPERTIES\n")
  file(APPEND "${warning_config_file}" "    ENVIRONMENT \"CDA_RAIL_WARNING_FILE=${warning_file}\"\n")
  file(APPEND "${warning_config_file}" "    FIXTURES_REQUIRED ${testname}_warnings_fixture)\n")
  file(APPEND "${warning_config_file}" "endforeach()\n")
  set_property(
    DIRECTORY
    APPEND
    PROPERTY TEST_INCLUDE_FILES "${warning_config_file}")

  add_test(NAME ${testname}_warning_summary COMMAND ${CMAKE_COMMAND} "-DWARNING_FILE=${warning_file}" -P
                                                    "${PROJECT_SOURCE_DIR}/cmake/PrintWarningSummary.cmake")
  set_tests_properties(${testname}_warning_summary PROPERTIES FIXTURES_CLEANUP ${testname}_warnings_fixture)

  set_target_properties(${testname} PROPERTIES FOLDER tests)
endmacro()
