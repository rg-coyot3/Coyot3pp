# This module tries to find libWebsockets library and include files
#
# LIBWEBSOCKETS_INCLUDE_DIR, path where to find libwebsockets.h
# LIBWEBSOCKETS_LIBRARY_DIR, path where to find libwebsockets.so
# LIBWEBSOCKETS_LIBRARIES, the library to link against
# LIBWEBSOCKETS_FOUND, If false, do not try to use libWebSockets
#
# This currently works probably only for Linux

FIND_PATH ( LIBWEBSOCKETS_INCLUDE_DIR libwebsockets.h
    /usr/include
    /usr/local/include

)
#SET(CMAKE_FIND_LIBRARY_SUFFIXES .a .so .dylib)
SET(CMAKE_FIND_LIBRARY_SUFFIXES .so)
FIND_LIBRARY ( LIBWEBSOCKETS_LIBRARIES websockets
    /usr/lib
    /usr/local/lib

)
message(" - find-libwebsockets-result : [${LIBWEBSOCKETS_LIBRARIES}]")
GET_FILENAME_COMPONENT( LIBWEBSOCKETS_LIBRARY_DIR ${LIBWEBSOCKETS_LIBRARIES} PATH )

SET ( LIBWEBSOCKETS_FOUND "NO" )
IF ( LIBWEBSOCKETS_INCLUDE_DIR )
    IF ( LIBWEBSOCKETS_LIBRARIES )
        SET ( LIBWEBSOCKETS_FOUND "YES" )
    ENDIF ( LIBWEBSOCKETS_LIBRARIES )
ENDIF ( LIBWEBSOCKETS_INCLUDE_DIR )

MARK_AS_ADVANCED(
    LIBWEBSOCKETS_LIBRARY_DIR
    LIBWEBSOCKETS_INCLUDE_DIR
    LIBWEBSOCKETS_LIBRARIES
)