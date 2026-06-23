# set required cmake version
cmake_minimum_required(VERSION 3.20)

# Check for xcodebuild and its version, even if not using Xcode generator
if(APPLE)
  find_program(XCODEBUILD_EXECUTABLE xcodebuild)
  if(XCODEBUILD_EXECUTABLE)
    execute_process(
      COMMAND "${XCODEBUILD_EXECUTABLE}" -version
      OUTPUT_VARIABLE XCODE_VERSION_OUTPUT
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCH "Xcode ([0-9]+)\\.([0-9]+)" _ ${XCODE_VERSION_OUTPUT})
    set(XCODE_VERSION_MAJOR "${CMAKE_MATCH_1}")
    set(XCODE_VERSION_MINOR "${CMAKE_MATCH_2}")
    math(EXPR XCODE_VERSION "${XCODE_VERSION_MAJOR} * 100 + ${XCODE_VERSION_MINOR}")
    if(XCODE_VERSION LESS 1600)
      message(FATAL_ERROR "Xcode 16.0 or newer is required. Detected: ${XCODE_VERSION_OUTPUT}")
    endif()
  else()
    message(
      WARNING "xcodebuild not found. Some features may not work as expected. If so, try installing Xcode >= 16.0.")
  endif()
endif()

# This avoids googletest complaining that this (IPO) policy is not set
cmake_policy(SET CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)

project(
  rail
  LANGUAGES CXX
  DESCRIPTION "")

# Require C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Highly recommended to prevent GCC/Clang-specific extensions

# Explicitly check for C++23 capable compilers
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "13.0")
    message(
      FATAL_ERROR
        "GCC >= 13.0 is strictly required for C++23 <ranges> support.
        You are using GCC ${CMAKE_CXX_COMPILER_VERSION}.
        Please update your compiler or use 'export CXX=g++-13'.
        Alternatively, you can use 'CXX=g++-13 CC=gcc-13 cmake -S . -B build -DCMAKE_BUILD_TYPE=Release'.")
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "17.0")
    message(
      FATAL_ERROR
        "Clang >= 17.0 is strictly required for C++23 <ranges> support.
        You are using Clang ${CMAKE_CXX_COMPILER_VERSION}.
        Please update your compiler.")
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "16.0")
    message(
      FATAL_ERROR
        "Apple Clang >= 16.0 (Xcode >= 16.0) is strictly required for C++23 <ranges> support.
        You are using Apple Clang ${CMAKE_CXX_COMPILER_VERSION}.
        Please update Xcode.")
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.34")
    message(
      FATAL_ERROR
        "MSVC >= 19.34 (Visual Studio 2022 17.4) is strictly required for C++23 <ranges> support.
        You are using MSVC ${CMAKE_CXX_COMPILER_VERSION}.
        Please update Visual Studio.")
  endif()
endif()

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
check_submodule_present("cli11")
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
