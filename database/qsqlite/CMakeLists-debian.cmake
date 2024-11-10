
find_package(Qt5 REQUIRED Sql)

set(LCOYOT3PPQSQLT_EXTERNAL_DEPENDENCES
  ${Qt5Sql_LIBRARIES}
  Cor3
)

qt5_wrap_cpp(MOC_LIB_SOURCES 
  ${LC3_MOCS_QSQLT}
)
set(LC3_SRCS_QSQLT ${LC3_SRCS_QSQLT} ${MOC_LIB_SOURCES})

add_library(${COYOT3PPCOMPONENT} ${LC3_SRCS_QSQLT})
add_library(${PROJECT_NAME}::${COYOT3PPCOMPONENT} ALIAS ${COYOT3PPCOMPONENT})
set_target_properties(${COYOT3PPCOMPONENT} 
        PROPERTIES VERSION ${PROJECT_VERSION}
                    SOVERSION ${PROJECT_VERSION_MAJOR})

target_link_libraries(${COYOT3PPCOMPONENT}
                    ${LCOYOT3PPQSQLT_EXTERNAL_DEPENDENCES}
                    Cor3
)

cmak3_make_package()

if(LCY_BUILD_WITH_MINIMAL_EXAMPLES)
  qt_wrap_cpp(MOC_EXAMPLE_SOURCES
  ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/Qsqlit3/qsqlite_connector_example.hpp
  )
  add_executable(qsqlit3_example
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/Qsqlit3/qsqlite_connector_example.cpp
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/Qsqlit3/qsqlite_example_main.cpp
    ${MOC_EXAMPLE_SOURCES})
  
  target_link_libraries(qsqlit3_example
    ${COYOT3PPCOMPONENT}
    Cor3
  )
endif()

