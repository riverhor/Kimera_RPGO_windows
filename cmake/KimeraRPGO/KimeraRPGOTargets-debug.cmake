#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "KimeraRPGO" for configuration "Debug"
set_property(TARGET KimeraRPGO APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(KimeraRPGO PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/KimeraRPGO.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS KimeraRPGO )
list(APPEND _IMPORT_CHECK_FILES_FOR_KimeraRPGO "${_IMPORT_PREFIX}/lib/KimeraRPGO.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
