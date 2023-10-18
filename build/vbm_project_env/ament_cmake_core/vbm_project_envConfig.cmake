# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vbm_project_env_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vbm_project_env_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vbm_project_env_FOUND FALSE)
  elseif(NOT vbm_project_env_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vbm_project_env_FOUND FALSE)
  endif()
  return()
endif()
set(_vbm_project_env_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vbm_project_env_FIND_QUIETLY)
  message(STATUS "Found vbm_project_env: 0.0.0 (${vbm_project_env_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vbm_project_env' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${vbm_project_env_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vbm_project_env_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${vbm_project_env_DIR}/${_extra}")
endforeach()
