
pkg_check_modules(Sqlit3 sqlite3 REQUIRED)

set(LCOYOT3PPSQLT3_EXTERNAL_DEPENDENCES
  ${Sqlit3_LIBRARIES}
)


add_library(${COYOT3PPCOMPONENT} ${LC3_SRCS_SQLT3})
add_library(${PROJECT_NAME}::${COYOT3PPCOMPONENT} ALIAS ${COYOT3PPCOMPONENT})
set_target_properties(${COYOT3PPCOMPONENT} 
        PROPERTIES VERSION ${PROJECT_VERSION}
                    SOVERSION ${PROJECT_VERSION_MAJOR})

target_link_libraries(${COYOT3PPCOMPONENT}
                    ${LCOYOT3PPSQLT3_EXTERNAL_DEPENDENCES}
                    Cor3
)


cmak3_make_package()

if(LCY_BUILD_WITH_MINIMAL_EXAMPLES)
  add_executable(sqlit3_example
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/Sqlit3/sqlit3_example.cpp
  )
  target_link_libraries(sqlit3_example
    ${COYOT3PPCOMPONENT}
    Cor3
  )

  add_executable(sqlit3_macros_example_1
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/Sqlit3/sqlit3_macros_example_1.cpp
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/Sqlit3/sqlit3_macro_classes.cpp
  )
  target_link_libraries(sqlit3_macros_example_1
    ${COYOT3PPCOMPONENT}
    Cor3  
  )

  add_executable(sqlit3_macros_example_2
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/Sqlit3/sqlit3_macros_example_2.cpp
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/Sqlit3/sqlit3_macro_classes.cpp
  )
  target_link_libraries(sqlit3_macros_example_2
    ${COYOT3PPCOMPONENT}
    Cor3  
  )


endif()