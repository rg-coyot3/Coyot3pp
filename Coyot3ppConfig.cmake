foreach(component ${Coyot3pp_FIND_COMPONENTS})
   include(${CMAKE_CURRENT_LIST_DIR}/Coyot3pp/Coyot3pp${component}Config.cmake)
endforeach()