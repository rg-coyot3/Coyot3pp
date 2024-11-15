set(COYOT3PPCOMPONENT          Cor3)
set(COYOT3PPCOMPONENTVERSION   1.0)


# base

pkg_check_modules( JsonCPP    jsoncpp   REQUIRED)
pkg_check_modules( YamlCPP    yaml-cpp  REQUIRED)
#pkg_check_modules( Oping      oping     REQUIRED)

set(Pthread_LIBRARIES       pthread)
set(Oping_LIBRARIES         oping)
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib/)
find_package(GeographicLib REQUIRED)
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/RapidJSON/)
find_package(RapidJSON REQUIRED)


file(GLOB LC3_SRCS_BASE 
            "${LC3_SRCS_BASE}"
            src/${PROJECT_NAME}/${COYOT3PPCOMPONENT}/base/*.cpp
            src/${PROJECT_NAME}/${COYOT3PPCOMPONENT}/module/*.cpp
)



include_directories(include)


set(LCOYOT3PPCORE_EXTERNAL_DEPENDENCES
  ${JsonCPP_LIBRARIES}
  ${Pthread_LIBRARIES}
  ${Oping_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${YamlCPP_LIBRARIES}
)








# BASE - BEGIN
add_library(${COYOT3PPCOMPONENT} 
              ${LC3_SRCS_BASE})

add_library(${PROJECT_NAME}::${COYOT3PPCOMPONENT} ALIAS ${COYOT3PPCOMPONENT})


set_target_properties(${COYOT3PPCOMPONENT} 
        PROPERTIES VERSION ${PROJECT_VERSION}
                    SOVERSION ${PROJECT_VERSION_MAJOR})

target_link_libraries(${COYOT3PPCOMPONENT}
             ${LCOYOT3PPCORE_EXTERNAL_DEPENDENCES})


set(CMAKE_INSTALL_PREFIX ${COYOT3_INSTALL_PREFIX})
install(  DIRECTORY   ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/${COYOT3PPCOMPONENT}
          DESTINATION include/${PROJECT_NAME})




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
               @ONLY
)

write_basic_package_version_file(
        "${PROJECT_NAME}${COYOT3PPCOMPONENT}ConfigVersion.cmake"
        VERSION ${COYOT3PPCOMPONENTVERSION}
        COMPATIBILITY AnyNewerVersion
)

# file(WRITE    ${CMAKE_BINARY_DIR}/${PROJECT_NAME}${COYOT3PPCOMPONENT}Config.cmake
#            "include(CmakeFindDependencyMacro) \n")
# file(APPEND   ${CMAKE_BINARY_DIR}/${PROJECT_NAME}${COYOT3PPCOMPONENT}Config.cmake
#            "include(${COYOT3PPCOMPONENT}Config.cmake) \n")

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
  add_executable(base_hello_world
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/${COYOT3PPCOMPONENT}/hello-world.cpp
  )
  target_link_libraries(base_hello_world
                  ${COYOT3PPCOMPONENT}
                  ${LCOYOT3PPCORE_EXTERNAL_DEPENDENCES}
  )
  add_executable(example_cvalue
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/${COYOT3PPCOMPONENT}/value-example.cpp
  )
  target_link_libraries(example_cvalue
    ${COYOT3PPCOMPONENT}
    ${LCOYOT3PPCORE_EXTERNAL_DEPENDENCES}
  )

  add_executable(example_modelclasses
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/${COYOT3PPCOMPONENT}/model-class-macros.cpp
  )
  target_link_libraries(example_modelclasses
    ${COYOT3PPCOMPONENT}
    ${LCOYOT3PPCORE_EXTERNAL_DEPENDENCES}
  )

  add_executable(example_module
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/${COYOT3PPCOMPONENT}/module-class-example.cpp
  )
  target_link_libraries(example_module
    ${COYOT3PPCOMPONENT}
    ${LCOYOT3PPCORE_EXTERNAL_DEPENDENCES}
  )
endif()

#BASE - END