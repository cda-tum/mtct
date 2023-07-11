set(_GUROBI_KNOWN_VERSIONS "1002" "1001")

find_path(GUROBI_INCLUDE_DIRS
  NAMES gurobi_c.h gurobi_c++.h
  HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME} ${PROJECT_SOURCE_DIR}/gurobi${_GUROBI_KNOWN_VERSIONS}
  PATH_SUFFIXES include linux64/include win64/include mac64/include)

find_library(GUROBI_LIBRARY
  NAMES gurobi gurobi100 gurobi100
  HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME} ${PROJECT_SOURCE_DIR}/gurobi${_GUROBI_KNOWN_VERSIONS}
  PATH_SUFFIXES lib linux64/lib win64/lib mac64/lib)


if(CXX)
  if(MSVC)
    set(MSVC_YEAR "2017")
    
    if(MT)
      set(M_FLAG "mt")
    else()
      set(M_FLAG "md")
    endif()
    
    find_library(GUROBI_CXX_LIBRARY
      NAMES gurobi_c++${M_FLAG}${MSVC_YEAR}
      HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
      PATH_SUFFIXES lib)
    find_library(GUROBI_CXX_DEBUG_LIBRARY
      NAMES gurobi_c++${M_FLAG}d${MSVC_YEAR}
      HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
      PATH_SUFFIXES lib)
  else()
    find_library(GUROBI_CXX_LIBRARY
      NAMES gurobi_c++
      HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
      PATH_SUFFIXES lib)
    set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gurobi DEFAULT_MSG GUROBI_LIBRARY)
