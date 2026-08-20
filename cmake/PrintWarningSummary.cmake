if(NOT DEFINED WARNING_FILE)
  message(FATAL_ERROR "WARNING_FILE was not provided")
endif()

if(NOT EXISTS "${WARNING_FILE}")
  return()
endif()

file(READ "${WARNING_FILE}" warning_content)

if("${warning_content}" STREQUAL "")
  file(REMOVE "${WARNING_FILE}")
  return()
endif()

message("\n[ WARNING SUMMARY ]")
message("${warning_content}")

file(REMOVE "${WARNING_FILE}")
