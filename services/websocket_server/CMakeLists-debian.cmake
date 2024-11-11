

find_package(LibWebsockets REQUIRED)
include_directories(${LibWebSockets_INCLUDE_DIRS})



set(LCOYOT3PPLWSS_EXTERNAL_DEPENDENCES
  ${LIBWEBSOCKETS_LIBRARIES}
)


add_library(${COYOT3PPCOMPONENT} ${LC3_SRCS_LWSS})
add_library(${PROJECT_NAME}::${COYOT3PPCOMPONENT} ALIAS ${COYOT3PPCOMPONENT})
set_target_properties(${COYOT3PPCOMPONENT} 
        PROPERTIES VERSION ${PROJECT_VERSION}
                    SOVERSION ${PROJECT_VERSION_MAJOR})

target_link_libraries(${COYOT3PPCOMPONENT}
                    ${LCOYOT3PPLWSS_EXTERNAL_DEPENDENCES}
                    Cor3
)


cmak3_make_package()


if(LCY_BUILD_WITH_MINIMAL_EXAMPLES)
  add_compile_definitions(LC3_EXAMPLE_HTML_CONTENT="${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/LwsServ3r/www")
  add_executable(lws_gateway_example
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/LwsServ3r/lws-server-wrapper-for-rest-server-example.cpp
  )
  
  target_link_libraries(lws_gateway_example
    ${COYOT3PPCOMPONENT}
    Cor3
  )
endif()

