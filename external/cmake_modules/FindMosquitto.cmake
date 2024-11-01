# - Find libMosquitto
# Find the native libMosquitto includes and libraries
#
#  Mosquitto_INCLUDE_DIR - where to find Mosquitto.h, etc.
#  Mosquitto_LIBRARIES   - List of libraries when using libMosquitto.
#  Mosquitto_FOUND       - True if libMosquitto found.

if (NOT Mosquitto_INCLUDE_DIR)
  find_path(Mosquitto_INCLUDE_DIR mosquitto.h)
endif()

if (NOT Mosquitto_LIBRARY)
  find_library(
    Mosquitto_LIBRARY
    NAMES mosquitto)
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  Mosquitto DEFAULT_MSG
  Mosquitto_LIBRARY Mosquitto_INCLUDE_DIR)

message(STATUS "libMosquitto include dir: ${Mosquitto_INCLUDE_DIR}")
message(STATUS "libMosquitto: ${Mosquitto_LIBRARY}")
set(Mosquitto_LIBRARIES ${Mosquitto_LIBRARY})

mark_as_advanced(Mosquitto_INCLUDE_DIR Mosquitto_LIBRARY)