# macro to add a test executable for one of the project libraries
macro(PACKAGE_ADD_TEST testname linklibs)
  # create an executable in which the tests will be stored
  add_executable(${testname} ${ARGN})
  # link the Google test infrastructure and a default main function to the test executable.
  target_link_libraries(${testname} PRIVATE ${linklibs} gmock gtest_main)

  set(warning_file "${CMAKE_CURRENT_BINARY_DIR}/${testname}_warnings.log")
  file(REMOVE "${warning_file}")

  # discover tests
  gtest_discover_tests(
    ${testname}
    TEST_LIST ${testname}_test_list
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" DISCOVERY_TIMEOUT 60)

  set_tests_properties(${${testname}_test_list} PROPERTIES ENVIRONMENT "CDA_RAIL_WARNING_FILE=${warning_file}"
                                                           FIXTURES_REQUIRED ${testname}_warnings_fixture)

  add_test(NAME ${testname}_warning_summary COMMAND ${CMAKE_COMMAND} "-DWARNING_FILE=${warning_file}" -P
                                                    "${PROJECT_SOURCE_DIR}/cmake/PrintWarningSummary.cmake")
  set_tests_properties(${testname}_warning_summary PROPERTIES FIXTURES_CLEANUP ${testname}_warnings_fixture)

  set_target_properties(${testname} PROPERTIES FOLDER tests)
endmacro()
