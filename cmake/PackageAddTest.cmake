# macro to add a test executable for one of the project libraries
macro(PACKAGE_ADD_TEST testname linklibs)
  # create an executable in which the tests will be stored
  add_executable(${testname} ${ARGN})
  # link the Google test infrastructure and a default main function to the test executable.
  target_link_libraries(${testname} PRIVATE ${linklibs} gmock gtest_main)
  # On macOS generate a .dSYM bundle so cpptrace can resolve symbols reliably.
  if(APPLE)
    add_custom_command(
      TARGET ${testname}
      POST_BUILD
      COMMAND dsymutil $<TARGET_FILE:${testname}>
      COMMENT "Generating ${testname}.dSYM")
  endif()
  # discover tests
  gtest_discover_tests(
    ${testname}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" DISCOVERY_TIMEOUT 60)
  set_target_properties(${testname} PROPERTIES FOLDER tests)
endmacro()
