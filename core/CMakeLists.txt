CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

# ----------------------------------------------------------------------------
# version.h
# Automatic version detection from Git

find_package(Git QUIET)
if(GIT_FOUND)
    # Getting the version description: v1.2.3-4-g5b7c2d
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --always --long
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_DESCRIBE_RAW
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    # Parsing a string (for example, v1.2.4-5-gabcdef)
    # The regular expression matches: 1.2.4 (version) and 5 (number of commits)
    if(GIT_DESCRIBE_RAW MATCHES "v?([0-9]+)\\.([0-9]+)\\.([0-9]+)-([0-9]+)-g([0-9a-f]+)")
        set(PROJECT_VERSION_MAJOR ${CMAKE_MATCH_1})
        set(PROJECT_VERSION_MINOR ${CMAKE_MATCH_2})
        set(PROJECT_VERSION_PATCH ${CMAKE_MATCH_3})
        set(GIT_COMMITS           ${CMAKE_MATCH_4})
        set(PROJECT_GIT_HASH      ${CMAKE_MATCH_5})
        
        set(GIT_TAG_VER "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
    else()
        # If there are no tags, we take only the hash
        set(PROJECT_GIT_HASH ${GIT_DESCRIBE_RAW})
    endif()
else()
    set(FINAL_VERSION "0.0.0-unknown")
endif()


# Initialize the project with the computed version
project(core VERSION ${GIT_TAG_VER} LANGUAGES CXX)

# Forming a string for About (PROJECT_FULL_VERSION_STR)
if(GIT_COMMITS EQUAL 0)
    set(PROJECT_FULL_VERSION_STR "${PROJECT_VERSION}")
else()
    set(PROJECT_FULL_VERSION_STR "${PROJECT_VERSION}-dev${GIT_COMMITS}")
endif()

message ("GENERATED PROJECT_FULL_VERSION_STR=${PROJECT_FULL_VERSION_STR}")

configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/version.h"
)


# ----------------------------------------------------------------------------
# core library

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if( NOT CMAKE_CXX_STANDARD )
	set (CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


file(GLOB_RECURSE CURRENT_SOURCES *.c *.cc *.cpp) 
file(GLOB_RECURSE CURRENT_HEADERS *.h *.hpp)

add_library(${PROJECT_NAME} STATIC
	${CURRENT_HEADERS}
	${CURRENT_SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# ----------------------------------------------------------------------------
