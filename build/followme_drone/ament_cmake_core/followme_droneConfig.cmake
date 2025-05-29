# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_followme_drone_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED followme_drone_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(followme_drone_FOUND FALSE)
  elseif(NOT followme_drone_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(followme_drone_FOUND FALSE)
  endif()
  return()
endif()
set(_followme_drone_CONFIG_INCLUDED TRUE)

# output package information
if(NOT followme_drone_FIND_QUIETLY)
  message(STATUS "Found followme_drone: 0.0.0 (${followme_drone_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'followme_drone' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${followme_drone_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(followme_drone_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${followme_drone_DIR}/${_extra}")
endforeach()
