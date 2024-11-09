
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
add_compile_definitions(IMAG3_EXAMPLES_DIRECTORY="${CMAKE_CURRENT_LIST_DIR}/doc/images")

add_library(${COYOT3PPCOMPONENT} ${LC3_SRCS_IMG3})
add_library(${PROJECT_NAME}::${COYOT3PPCOMPONENT} ALIAS ${COYOT3PPCOMPONENT})

set_target_properties(${COYOT3PPCOMPONENT} 
        PROPERTIES VERSION ${PROJECT_VERSION}
                    SOVERSION ${PROJECT_VERSION_MAJOR})

target_link_libraries(${COYOT3PPCOMPONENT}
                    ${LC3_IMG3_REQ_LIBS}
                    Cor3
)

cmak3_make_package()



if(LCY_BUILD_WITH_MINIMAL_EXAMPLES)
  add_executable(imag3_lib_example
    ${COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR}/Imag3/imag3_lib_example.cpp)
  
  target_link_libraries(imag3_lib_example
    ${COYOT3PPCOMPONENT}
    Cor3
  )
endif()

