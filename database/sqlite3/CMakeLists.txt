
set(COYOT3PPCOMPONENT          Sqlit3)
set(COYOT3PPCOMPONENTVERSION   1.0)

include_directories(include)
include_directories(${COYOT3PPCOR3_INCLUDE_DIRS})


file(GLOB LC3_SRCS_SQLT3
          src/${PROJECT_NAME}/${COYOT3PPCOMPONENT}/Sqlit3Connector/*.cpp
)



if(WIN32)
  message( FATAL_ERROR "SQLT3 - not previewed for windows.")
elseif(UNIX)
  message( "SQLT3 - gnu system detected.")
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
  message("SQLT3 - gnu environment is ${LSB_RELEASE_ID_SHORT} ${LSB_RELEASE_NAME_SHORT} / ${LSB_RELEASE_NAME_SHORT}")
  include(CMakeLists-debian.cmake)
else()
  message( FATAL_ERROR "SQLT3 - ENVIRONMENT NOT DETECTED")
endif()
