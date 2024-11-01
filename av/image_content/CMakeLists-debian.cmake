
find_package(OpenCV 4 REQUIRED)
find_package(X11)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${X11_INCLUDE_DIRS}
)
add_definitions(
  ${OpenCV_DEFINITIONS}
  ${X11_DEFINITIONS}
)

set(LC3_IMG3_REQ_LIBS
  ${OpenCV_LIBRARIES}
  ${X11_LIBRARIES}
  libX11.so
)

add_library(${COYOT3PPCOMPONENT} ${LC3_SRCS_IMG3})
add_library(${PROJECT_NAME}::${COYOT3PPCOMPONENT} ALIAS ${COYOT3PPCOMPONENT})

set_target_properties(${COYOT3PPCOMPONENT} 
        PROPERTIES VERSION ${PROJECT_VERSION}
                    SOVERSION ${PROJECT_VERSION_MAJOR})

target_link_libraries(${COYOT3PPCOMPONENT}
                    ${LC3_IMG3_REQ_LIBS}
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

