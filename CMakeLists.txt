cmake_minimum_required(VERSION 3.15)
project(Coyot3pp LANGUAGES CXX VERSION 1.0)



option(LCY_BUILD_WITH_MINIMAL_EXAMPLES   "Build with minimal examples"       ON)
option(LCY_BUILD_COMPONENT_REST          "Build with R3st components"        ON)
option(LCY_BUILD_COMPONENT_MQTT          "Build with Mqtt components"        ON)
option(LCY_BUILD_COMPONENT_POSTGRESQL    "Build with PostgreSql components"  ON)
option(LCY_BUILD_COMPONENT_QSQLITE       "Build with QSqlite    components"  ON)
option(LCY_BUILD_COMPONENT_IMAG3         "Build with Imag3      components"  ON)
OPTION(LCY_BUILD_COMPONENT_H264RTSP      "Build with H264Rtsp   components"  ON)

set(CMAKE_CXX_STANDARD 23)
SET (CMAKE_VERBOSE_MAKEFILE 1)
set(CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type")


if(NOT DEFINED COYOT3_INSTALL_PREFIX)
  set(COYOT3_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install/coyot3pp)
endif()


set(CMAKE_INSTALL_PREFIX ${COYOT3_INSTALL_PREFIX})
message("install prefix set to : ${CMAKE_INSTALL_PREFIX}")



find_package(PkgConfig REQUIRED)
include(CheckFunctionExists)
include(CheckSymbolExists)
include(CheckIncludeFile)
include(CheckIncludeFiles)
include(CheckLibraryExists)
include(CheckTypeSize)
include(CheckCSourceCompiles)
include(CMakePackageConfigHelpers)
find_package(PkgConfig REQUIRED)

include(CMakeLists-coyot3-commons.cmake)


list(APPEND CMAKE_MODULE_PATH 
  "${CMAKE_CURRENT_LIST_DIR}/external/cmake_modules")
list(APPEND CMAKE_MODULE_PATH "/usr/share")


set(COYOT3PP_MINIMAL_EXAMPLES_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/minimal-examples)
set(COYOT3PPCOR3_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/cor3/include)
set(COYOT3PPIMG3_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/av/image_content/include)
message(" coyot3ppcore include dir : ${COYOT3PPCOR3_INCLUDE_DIRS}")

# COMPONENTS DIRS INCLUDE - begin

add_subdirectory(cor3)

if(LCY_BUILD_COMPONENT_REST)
  add_subdirectory(communication/rest_connector)
endif()
if(LCY_BUILD_COMPONENT_MQTT)
  add_subdirectory(communication/mqtt)
endif()
if(LCY_BUILD_COMPONENT_POSTGRESQL)
  add_subdirectory(database/postgresql)
endif()
if(LCY_BUILD_COMPONENT_QSQLITE)
  add_subdirectory(database/qsqlite)
endif()
if(LCY_BUILD_COMPONENT_IMAG3 OR LCY_BUILD_COMPONENT_H264RTSP)
  add_subdirectory(av/image_content)
endif()
if(LCY_BUILD_COMPONENT_H264RTSP)
  add_subdirectory(av/rtsp_server)
endif()
# COMPONENTS DIRS INCLUDE - end


file(WRITE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake 
          "foreach(component \$\{${PROJECT_NAME}_FIND_COMPONENTS\})\n"
          "   include(\$\{CMAKE_CURRENT_LIST_DIR\}/${PROJECT_NAME}/${PROJECT_NAME}\$\{component\}Config.cmake)\n"
          "endforeach()")
          
configure_file( ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake
                "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
                @ONLY)       

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES           
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION     lib/cmake
)


add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)