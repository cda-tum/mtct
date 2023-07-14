set(GUROBI_HOME
    ""
    CACHE PATH "Home of the gurobi distribution.")
if(DEFINED ENV{GUROBI_HOME})
  set(GUROBI_HOME $ENV{GUROBI_HOME})
  message(STATUS "GUROBI_HOME from environment: ${GUROBI_HOME}")
endif()

if(GUROBI_HOME STREQUAL "")
  if(UNIX AND NOT APPLE)
    set(GUROBI_HOME_DEFAULT_PATHS "/home/opt/gurobi*" "/opt/gurobi*")
    set(GUROBI_PLATFORM "linux64")
  elseif(APPLE)
    set(GUROBI_HOME_DEFAULT_PATHS "/Library/gurobi*")
    set(GUROBI_PLATFORM "macos_universal2")
  elseif(WIN32)
    set(GUROBI_HOME_DEFAULT_PATHS "C:/gurobi*")
    set(GUROBI_PLATFORM "win64")
  endif()

  # Append the project root to the default paths
  list(APPEND GUROBI_HOME_DEFAULT_PATHS "${PROJECT_SOURCE_DIR}/gurobi*")
  foreach(path IN LISTS GUROBI_HOME_DEFAULT_PATHS)
    file(GLOB GUROBI_HOME_DEFAULT_PATH ${path}/${GUROBI_PLATFORM})
    if(NOT GUROBI_HOME_DEFAULT_PATH)
      continue()
    endif()
    list(SORT GUROBI_HOME_DEFAULT_PATH)
    list(GET GUROBI_HOME_DEFAULT_PATH -1 GUROBI_HOME)
    # Eagerly abort on the first found library
    if(GUROBI_HOME)
      break()
    endif()
  endforeach()
  if(NOT GUROBI_HOME STREQUAL "")
    message(STATUS "GUROBI_HOME from default path: ${GUROBI_HOME}")
  endif()
endif()

if(GUROBI_HOME STREQUAL "")
  message(WARNING "Could not find GUROBI_HOME. Please set it manually.")
endif()

if(UNIX AND NOT APPLE)
  set(LIBRARY_EXTENSION so)
  set(LIBRARY_PATH_SUFFIXES lib bin)
elseif(APPLE)
  set(LIBRARY_EXTENSION dylib)
  set(LIBRARY_PATH_SUFFIXES lib bin)
elseif(WIN32)
  set(LIBRARY_EXTENSION dll)
  set(LIBRARY_PATH_SUFFIXES bin)
endif()

foreach(suffix IN LISTS LIBRARY_PATH_SUFFIXES)
  set(GLOB_EXPRESSION "${GUROBI_HOME}/${suffix}/*gurobi*.${LIBRARY_EXTENSION}")
  file(GLOB TEMP_LIBRARIES ${GLOB_EXPRESSION})
  foreach(library ${TEMP_LIBRARIES})
    # The `_light` library is a stripped down version of the library that is not used by us
    if(NOT ${library} MATCHES "_light")
      list(APPEND LIBRARIES ${library})
    endif()
  endforeach()
endforeach()

# Get the last library (latest version)
list(SORT LIBRARIES)
list(GET LIBRARIES -1 GUROBI_LIBRARY)
mark_as_advanced(GUROBI_LIBRARY)

# on Windows there is a separate import library in the /lib directory
if(WIN32)
  get_filename_component(GUROBI_LIBRARY_NAME ${GUROBI_LIBRARY} NAME_WE)
  find_library(
    GUROBI_IMPLIB
    NAMES ${GUROBI_LIBRARY_NAME}
    PATHS ${GUROBI_HOME}
    PATH_SUFFIXES lib)
  mark_as_advanced(GUROBI_IMPLIB)
endif()

find_path(
  GUROBI_INCLUDE_DIR
  NAMES gurobi_c++.h gurobi_c.h
  PATHS ${GUROBI_HOME}
  PATH_SUFFIXES include
  NO_DEFAULT_PATH)
mark_as_advanced(GUROBI_INCLUDE_DIR)

if(GUROBI_LIBRARY AND NOT TARGET Gurobi::GurobiC)
  add_library(Gurobi::GurobiC SHARED IMPORTED)
  target_include_directories(Gurobi::GurobiC INTERFACE ${GUROBI_INCLUDE_DIR})
  set_target_properties(Gurobi::GurobiC PROPERTIES IMPORTED_LOCATION ${GUROBI_LIBRARY})
  if(GUROBI_IMPLIB)
    set_target_properties(Gurobi::GurobiC PROPERTIES IMPORTED_IMPLIB ${GUROBI_IMPLIB})
  endif()
endif()

# Gurobi ships with some compiled versions of its C++ library for specific compilers, however it also comes with the
# source code. We will compile the source code ourselves -- this is much safer, as it guarantees the same compiler
# version and flags. (Note: doing this is motivated by actual sometimes-subtle ABI compatibility bugs)

find_path(
  GUROBI_SRC_DIR
  NAMES "Model.h"
  PATHS "${GUROBI_HOME}/src/cpp/")
mark_as_advanced(GUROBI_SRC_DIR)

file(GLOB GUROBI_CXX_SRC CONFIGURE_DEPENDS ${GUROBI_SRC_DIR}/*.cpp)
if(TARGET Gurobi::GurobiC
   AND GUROBI_CXX_SRC
   AND NOT TARGET Gurobi::GurobiCXX)
  add_library(GurobiCXX STATIC EXCLUDE_FROM_ALL ${GUROBI_CXX_SRC})
  add_library(Gurobi::GurobiCXX ALIAS GurobiCXX)

  if(MSVC)
    target_compile_definitions(GurobiCXX PRIVATE "WIN64")
  endif()

  target_include_directories(GurobiCXX PUBLIC ${GUROBI_INCLUDE_DIR})
  target_link_libraries(GurobiCXX PUBLIC Gurobi::GurobiC)

  # We need to be able to link this into a shared library and set the SYSTEM flag for the include dirs of the Gurobi
  # libs to suppress warnings
  set_target_properties(
    GurobiCXX PROPERTIES POSITION_INDEPENDENT_CODE ON INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
                                                      $<TARGET_PROPERTY:GurobiCXX,INTERFACE_INCLUDE_DIRECTORIES>
  )# cmake-lint: disable=C0307

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gurobi DEFAULT_MSG GUROBI_LIBRARY GUROBI_INCLUDE_DIR GUROBI_SRC_DIR)
