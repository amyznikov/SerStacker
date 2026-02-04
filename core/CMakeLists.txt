CMAKE_MINIMUM_REQUIRED(VERSION 3.16)
project(core VERSION 1.1.1 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if( NOT CMAKE_CXX_STANDARD )
	set (CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  if(NOT CMAKE_TOOLCHAIN_FILE)
    # it _must_ go before any project in order to work
    set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation Directory")
  else()
    #Android: set output folder to ${CMAKE_BINARY_DIR}
    set(LIBRARY_OUTPUT_PATH_ROOT ${CMAKE_BINARY_DIR} CACHE PATH "root for library output, set this to change where android libs are compiled to" )
    # any cross-compiling
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
  endif()
endif()

# ----------------------------------------------------------------------------
# version.h

find_package(Git QUIET)
if(GIT_FOUND AND (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git" OR EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../.git" ) )
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE PROJECT_GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
else()
    set(PROJECT_GIT_HASH "unknown")
endif()


configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/version.h"
)

# ----------------------------------------------------------------------------
# target sources
file(GLOB_RECURSE CURRENT_SOURCES *.c *.cc *.cpp) 
file(GLOB_RECURSE CURRENT_HEADERS *.h *.hpp)

# ----------------------------------------------------------------------------

add_library(${PROJECT_NAME} STATIC
	${CURRENT_HEADERS}
	${CURRENT_SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# ----------------------------------------------------------------------------
