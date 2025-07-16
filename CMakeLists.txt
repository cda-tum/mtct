# set required cmake version
cmake_minimum_required(VERSION 3.20)

# Check for minimal Xcode version (e.g., 16.0)
if(APPLE AND CMAKE_GENERATOR STREQUAL "Xcode")
  execute_process(
    COMMAND xcodebuild -version
    OUTPUT_VARIABLE XCODE_VERSION_OUTPUT
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REGEX MATCH "Xcode ([0-9]+)\\.([0-9]+)" _ ${XCODE_VERSION_OUTPUT})
  set(XCODE_VERSION_MAJOR "${CMAKE_MATCH_1}")
  set(XCODE_VERSION_MINOR "${CMAKE_MATCH_2}")
  math(EXPR XCODE_VERSION "${XCODE_VERSION_MAJOR} * 100 + ${XCODE_VERSION_MINOR}")

  # Require Xcode 16.0 or newer
  if(XCODE_VERSION LESS 1600)
    message(FATAL_ERROR "Xcode 16.0 or newer is required. Detected: ${XCODE_VERSION_OUTPUT}")
  endif()
endif()

# This avoids googletest complaining that this (IPO) policy is not set
cmake_policy(SET CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(
  rail
  LANGUAGES CXX
  DESCRIPTION "")

# check whether the submodule ``modulename`` is correctly cloned in the ``/extern`` directory.
macro(CHECK_SUBMODULE_PRESENT modulename)
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/${modulename}/CMakeLists.txt")
    message(
      FATAL_ERROR
        "${modulename} submodule not cloned properly. \
        Please run `git submodule update --init --recursive` \
        from the main project directory")
  endif()
endmacro()

include(cmake/StandardProjectSettings.cmake)
if(NOT TARGET project_options)
  # Use the options specified in CompilerOptions.cmake
  add_library(project_options INTERFACE)

  # Standard compiler options
  include(cmake/CompilerOptions.cmake)
  enable_project_options(project_options)

  # Sanitizer options if supported by compiler
  include(cmake/Sanitizers.cmake)
  enable_sanitizers(project_options)
endif()

check_submodule_present("googletest")
check_submodule_present("json")
check_submodule_present("tinyxml2")
check_submodule_present("gsl")
check_submodule_present("plog")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

find_package(Gurobi REQUIRED)

if(NOT GUROBI_FOUND)
  message(WARNING "Did not find Gurobi")
endif()

# add main library code
add_subdirectory(src)

# add test code
option(BUILD_TESTS "Also build tests for rail project")
if(BUILD_TESTS)
  enable_testing()
  include(cmake/PackageAddTest.cmake)
  include(GoogleTest)
  add_subdirectory(test)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  add_subdirectory(apps)
endif()
