# - Try to find the LibPNG library
#
# This module defines the following variables
#
# PNG_FOUND - Was libpng found
# PNG_INCLUDE_DIRS - the PNG include directories
# PNG_LIBRARIES - Link to this
#
# This module accepts the following variables
#
# PNG_ROOT - Can be set to PNG install path or Windows build path
#

if (NOT DEFINED PNG_ROOT)
     set (PNG_ROOT /usr /usr/local)
endif (NOT DEFINED PNG_ROOT)

if(MSVC)
     set(LIB_PATHS ${PNG_ROOT} ${PNG_ROOT}/Release)
else(MSVC)
     set (LIB_PATHS ${PNG_ROOT} ${PNG_ROOT}/lib)
endif(MSVC)

macro(_FIND_PNG_LIBRARIES _var)
     find_library(${_var}
          NAMES
          ${ARGN}
          PATHS
          ${LIB_PATHS}
          PATH_SUFFIXES 
          lib
      )
     mark_as_advanced(${_var})
endmacro()

macro(_PNG_APPEND_LIBRARIES _list _release)
set(_debug ${_release}_DEBUG)
if(${_debug})
     set(${_list} ${${_list}} optimized ${${_release}} debug ${${_debug}})
else()
     set(${_list} ${${_list}} ${${_release}})
endif()
endmacro()

if(MSVC)
     find_path(PNG_INCLUDE_DIR NAMES png.h
          PATHS
          ${PNG_ROOT}/src/windows
          ${PNG_ROOT}/src/windows/libpng
     )
else(MSVC)
     # Linux/OS X builds
     find_path(PNG_INCLUDE_DIR NAMES png.h
     PATHS
     ${PNG_ROOT}/include/libpng
     )
endif(MSVC)

# Find the libraries
if(MSVC)
     _FIND_PNG_LIBRARIES(PNG_LIBRARIES libpng.lib)
else(MSVC)
     # Linux/OS X builds
     if(UNIX)
          _FIND_PNG_LIBRARIES(PNG_LIBRARIES libpng.so)
     endif(UNIX)
     if(APPLE)
          _FIND_PNG_LIBRARIES(PNG_LIBRARIES libpng.a)
     endif(APPLE)
endif(MSVC)

message("png library found at ${PNG_LIBRARIES}")

# handle the QUIETLY and REQUIRED arguments and set PNG_FOUND to TRUE if
# all listed variables are TRUE
include("${CMAKE_ROOT}/Modules/FindPackageHandleStandardArgs.cmake")
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Png DEFAULT_MSG
     PNG_LIBRARIES)

if(MSVC)
     string(REGEX REPLACE "/png$" "" VAR_WITHOUT ${PNG_INCLUDE_DIR})
     string(REGEX REPLACE "/windows$" "" VAR_WITHOUT ${VAR_WITHOUT})
     set(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIRS} "${VAR_WITHOUT}")
     string(REGEX REPLACE "/libgpng.lib" "" PNG_LIBRARIES_DIR ${PNG_LIBRARIES})
else(MSVC)
     # Linux/OS X builds
     set(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIR})
     string(REGEX REPLACE "/libpng.so" "" PNG_LIBRARIES_DIR ${PNG_LIBRARIES})
endif(MSVC)

if(PNG_FOUND)
      # _PNG_APPEND_LIBRARIES(PNG PNG_LIBRARIES)
endif()
