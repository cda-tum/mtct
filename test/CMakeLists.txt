if(NOT TARGET gtest OR NOT TARGET gmock)
    # Prevent overriding the parent project's compiler/linker settings on Windows
    set(gtest_force_shared_crt # cmake-lint: disable=C0103
            ON
            CACHE BOOL "" FORCE)
    add_subdirectory(
            ${PROJECT_SOURCE_DIR}/extern/googletest "extern/googletest" EXCLUDE_FROM_ALL)
endif()

macro(package_add_test TESTNAME)
    # create an exectuable in which the tests will be stored
    add_executable(${TESTNAME} ${ARGN})
    # link the Google test infrastructure and a default main fuction to the test executable.
    target_link_libraries(${TESTNAME} PRIVATE ${PROJECT_NAME} gmock gtest_main)
    # discover tests
    gtest_discover_tests(${TESTNAME} WORKING_DIRECTORY ${PROJECT_DIR} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_DIR}")
    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests CMAKE_CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
    add_custom_command(TARGET ${TESTNAME}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_DIR:${TESTNAME}>/${TESTNAME} ${CMAKE_BINARY_DIR}/${TESTNAME}
            COMMENT "Creating symlinks for ${TESTNAME}"
            VERBATIM)
endmacro()

package_add_test(
        ${PROJECT_NAME}_test
        test_railwaynetwork.cpp)