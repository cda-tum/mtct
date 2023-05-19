if(NOT TARGET gtest OR NOT TARGET gmock)
    # Prevent overriding the parent project's compiler/linker settings on Windows
    set(gtest_force_shared_crt # cmake-lint: disable=C0103
            ON
            CACHE BOOL "" FORCE)
    add_subdirectory(
            ${PROJECT_SOURCE_DIR}/extern/googletest "extern/googletest" EXCLUDE_FROM_ALL)
endif()

package_add_test(${PROJECT_NAME}_test ${PROJECT_NAME} test_railwaynetwork.cpp test_probleminstances.cpp test_multiarray.cpp test_gurobi_vss_gen.cpp test_helper.cpp)