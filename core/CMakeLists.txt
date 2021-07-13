CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

project(core CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if( NOT CMAKE_CXX_STANDARD )
	set (CMAKE_CXX_STANDARD 14)
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



# target sources
file(GLOB_RECURSE SOURCE_FILES 
  "*.c"
  "*.cc"
  "*.cpp")
 
file(GLOB_RECURSE HEADER_FILES
  "*.h"
  "*.hpp")

add_library(${PROJECT_NAME} STATIC
	${HEADER_FILES}
	${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CORE_INCLUDES} )

# remove duplicates from include direcories
get_property(__incdirs TARGET ${PROJECT_NAME} PROPERTY INCLUDE_DIRECTORIES)
list(REMOVE_DUPLICATES __incdirs)
set_property( TARGET ${PROJECT_NAME} PROPERTY INCLUDE_DIRECTORIES ${__incdirs})

# Print status
MESSAGE(STATUS "")
MESSAGE(STATUS "OpenCV_VERSION:  ${OpenCV_VERSION}")
MESSAGE(STATUS "")
