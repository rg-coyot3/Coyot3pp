
set(COYOT3PPCOMPONENT          QSqlit3)
set(COYOT3PPCOMPONENTVERSION   1.0)

include_directories(include)
include_directories(${COYOT3PPCOR3_INCLUDE_DIRS})


file(GLOB LC3_SRCS_QSQLT
          src/${PROJECT_NAME}/${COYOT3PPCOMPONENT}/QSqlit3Connector/*.cpp
)

if(WIN32)
  message( FATAL_ERROR "QSQLT - not previewed for windows.")
elseif(UNIX)
  message( "QSQLT - gnu system detected.")
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
  message("QSQLT - gnu environment is ${LSB_RELEASE_ID_SHORT} ${LSB_RELEASE_NAME_SHORT} / ${LSB_RELEASE_NAME_SHORT}")
  include(CMakeLists-debian.cmake)
else()
  message( FATAL_ERROR "QSQLT - ENVIRONMENT NOT DETECTED")
endif()
