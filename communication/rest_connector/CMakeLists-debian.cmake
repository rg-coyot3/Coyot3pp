
set(COYOT3PPCOMPONENT          R3stCli3nt)
set(COYOT3PPCOMPONENTVERSION   1.0)





include_directories(include)
include_directories(${COYOT3PPCOR3_INCLUDE_DIRS})
list(APPEND CMAKE_MODULE_PATH "/usr/share")
list(APPEND CMAKE_MODULE_PATH "/usr/lib/x86_64-linux-gnu")

set(LCOYOT3PPR3ST_EXTERNAL_DEPENDENCES
  curl
  curlpp
  Cor3
)

file(GLOB_RECURSE LC3_SRCS_R3ST
          src/${PROJECT_NAME}/*.cpp)



add_library(${COYOT3PPCOMPONENT} ${LC3_SRCS_R3ST})

add_library(${PROJECT_NAME}::${COYOT3PPCOMPONENT} ALIAS ${COYOT3PPCOMPONENT})

set_target_properties(${COYOT3PPCOMPONENT} 
        PROPERTIES VERSION ${PROJECT_VERSION}
                    SOVERSION ${PROJECT_VERSION_MAJOR})

target_link_libraries(${COYOT3PPCOMPONENT}
                    ${LCOYOT3PPR3ST_EXTERNAL_DEPENDENCES}
                    Cor3
)


set(CMAKE_INSTALL_PREFIX ${COYOT3_INSTALL_PREFIX})
install(  DIRECTORY   ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/${COYOT3PPCOMPONENT}
          DESTINATION include/${PROJECT_NAME}
)


install(  TARGETS     ${COYOT3PPCOMPONENT}

          EXPORT      ${PROJECT_NAME}${COYOT3PPCOMPONENT}Targets
          FILE_SET    HEADERS
          LIBRARY     DESTINATION     lib/${PROJECT_NAME}
          ARCHIVE     DESTINATION     lib/${PROJECT_NAME}
          RUNTIME     DESTINATION     bin/${PROJECT_NAME}
          INCLUDES    DESTINATION     include
)

install(  EXPORT      ${PROJECT_NAME}${COYOT3PPCOMPONENT}Targets
          FILE        ${PROJECT_NAME}${COYOT3PPCOMPONENT}Targets.cmake
          NAMESPACE   ${PROJECT_NAME}::
          DESTINATION lib/cmake/${PROJECT_NAME}
          COMPONENT   ${COYOT3PPCOMPONENT}
)


file(WRITE  ${CMAKE_BINARY_DIR}/${COYOT3PPCOMPONENT}Config.cmake.in "")
file(APPEND ${CMAKE_BINARY_DIR}/${COYOT3PPCOMPONENT}Config.cmake.in
            "include(\$\{CMAKE_CURRENT_LIST_DIR\}/${PROJECT_NAME}${COYOT3PPCOMPONENT}ConfigVersion.cmake)\n"
            "include(\$\{CMAKE_CURRENT_LIST_DIR\}/${PROJECT_NAME}${COYOT3PPCOMPONENT}Targets.cmake)\n"
)

configure_file(${CMAKE_BINARY_DIR}/${COYOT3PPCOMPONENT}Config.cmake.in
            "${CMAKE_BINARY_DIR}/${PROJECT_NAME}${COYOT3PPCOMPONENT}Config.cmake"
            @ONLY)

write_basic_package_version_file(
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}${COYOT3PPCOMPONENT}ConfigVersion.cmake
            VERSION ${COYOT3PPCOMPONENTVERSION}
            COMPATIBILITY AnyNewerVersion
)

install(FILES 
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}${COYOT3PPCOMPONENT}Config.cmake"
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}${COYOT3PPCOMPONENT}ConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
)



if(LCY_BUILD_WITH_MINIMAL_EXAMPLES)
  add_executable(rest_component_example
            ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/R3st/rest_client_example.cpp
  )
  target_link_libraries(rest_component_example
            ${COYOT3PPCOMPONENT}
            Cor3
  )
endif()