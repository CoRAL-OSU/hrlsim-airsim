#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hrlsim_interfaces::hrlsim_interfaces__rosidl_generator_c" for configuration "Debug"
set_property(TARGET hrlsim_interfaces::hrlsim_interfaces__rosidl_generator_c APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(hrlsim_interfaces::hrlsim_interfaces__rosidl_generator_c PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libhrlsim_interfaces__rosidl_generator_c.so"
  IMPORTED_SONAME_DEBUG "libhrlsim_interfaces__rosidl_generator_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hrlsim_interfaces::hrlsim_interfaces__rosidl_generator_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_hrlsim_interfaces::hrlsim_interfaces__rosidl_generator_c "${_IMPORT_PREFIX}/lib/libhrlsim_interfaces__rosidl_generator_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
