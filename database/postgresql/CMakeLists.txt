
if(WIN32)
  message( FATAL_ERROR "PSQL - not previewed for windows.")
elseif(UNIX)
  message( "PSQL - gnu system detected.")
  find_program(LSB_RELEASE_EXEC lsb_release)
  execute_process(COMMAND ${LSB_RELEASE_EXEC} -is
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  execute_process(COMMAND ${LSB_RELEASE_EXEC} -rs
    OUTPUT_VARIABLE LSB_RELEASE_VER_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  execute_process(COMMAND ${LSB_RELEASE_EXEC} -cs
    OUTPUT_VARIABLE LSB_RELEASE_NAME_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message("PSQL - gnu environment is ${LSB_RELEASE_ID_SHORT} ${LSB_RELEASE_NAME_SHORT} / ${LSB_RELEASE_NAME_SHORT}")
  include(CMakeLists-debian.cmake)
else()
  message( FATAL_ERROR "PSQL - ENVIRONMENT NOT DETECTED")
endif()
