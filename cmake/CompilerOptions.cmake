# set common compiler options for projects
function(enable_project_options target_name)
  # set required C++ standard and disable compiler specific extensions
  target_compile_features(${target_name} INTERFACE cxx_std_23)

  # Option to enable time tracing with clang
  if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    option(ENABLE_BUILD_WITH_TIME_TRACE "Enable -ftime-trace to generate time tracing .json files on clang" OFF)
    if(ENABLE_BUILD_WITH_TIME_TRACE)
      target_compile_options(${target_name} INTERFACE -ftime-trace)
    endif()
  endif()

  if(MSVC)
    target_compile_options(${target_name} INTERFACE /utf-8)
  else()
    # always include debug symbols (avoids common problems with LTO)
    target_compile_options(${target_name} INTERFACE -g)

    # Large test files exceed the default number of sections of a MinGW object file. Moreover, MinGW does not keep the
    # stack 32-byte aligned, yet GCC stores 32-byte stack objects with aligned AVX moves, which crash at random (GCC bug
    # 54412); the assembler hence replaces aligned vector moves by unaligned ones.
    if(MINGW)
      target_compile_options(${target_name} INTERFACE -Wa,-mbig-obj -Wa,-muse-unaligned-vector-move)
    endif()

    # enable coverage collection options
    option(ENABLE_COVERAGE "Enable coverage reporting for gcc/clang" FALSE)
    if(ENABLE_COVERAGE)
      target_compile_options(${target_name} INTERFACE --coverage -fprofile-arcs -ftest-coverage -O0)
      target_link_libraries(${target_name} INTERFACE gcov --coverage)
    endif()

    if(NOT DEPLOY)
      # only include machine-specific optimizations when building for the host machine
      target_compile_options(${target_name} INTERFACE -mtune=native)

      include(CheckCXXCompilerFlag)
      check_cxx_compiler_flag(-march=native HAS_MARCH_NATIVE)
      if(HAS_MARCH_NATIVE)
        target_compile_options(${target_name} INTERFACE -march=native)
      endif()
    endif()

    # enable some more optimizations in release mode
    #
    # No -ffinite-math-only: the code relies on infinity, e.g., as the maximal travel time of a train that may stop, and
    # the flag lets the compiler drop comparisons with it (see Definitions.hpp)
    target_compile_options(${target_name} INTERFACE $<$<CONFIG:RELEASE>:-fno-math-errno -fno-trapping-math>)

    # enable some more options for better debugging
    target_compile_options(${target_name} INTERFACE $<$<CONFIG:DEBUG>:-fno-omit-frame-pointer
                                                    -fno-optimize-sibling-calls -fno-inline-functions>)
  endif()

  option(BINDINGS "Configure for building Python bindings")
  if(BINDINGS)
    target_compile_options(${target_name} INTERFACE -fvisibility=hidden)
    include(CheckPIESupported)
    check_pie_supported()
    set_target_properties(${target_name} PROPERTIES INTERFACE_POSITION_INDEPENDENT_CODE ON)
  endif()
endfunction()
