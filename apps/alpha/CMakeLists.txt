CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

#return()

#find_path(LASLIB_INCLUDES include/laslib)
#find_library(LASLIB_LIBRARY laslib)

#message(STATUS "")

#if( LASLIB_INCLUDES AND LASLIB_LIBRARY ) 
#	set( LASLIB_INCLUDES "${LASLIB_INCLUDES}/include")
#    message(STATUS "laslib FOUND: LASLIB_INCLUDES=${LASLIB_INCLUDES} LASLIB_LIBRARY=${LASLIB_LIBRARY}")
#    add_definitions(-DHAVE_LASLIB=1)
#else()
#
#    message(WARNING "Can not locate laslib library under search parth.
#      Install please lastools package: https://github.com/LAStools/LAStools.git")
#      
#    message(WARNING "LASLIB_INCLUDES=${LASLIB_INCLUDES} LASLIB_LIBRARY=${LASLIB_LIBRARY}")
#    
#    return()
#endif()


project(alpha CXX)

if( NOT CMAKE_CXX_STANDARD )
	set (CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
set(CMAKE_LINK_WHAT_YOU_USE OFF)


set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)

file(GLOB_RECURSE SOURCE_FILES
	${SRC}/*.c
	${SRC}/*.cc
	${SRC}/*.cpp)

file(GLOB_RECURSE HEADER_FILES
	${SRC}/*.h
	${SRC}/*.hpp)

set(APP_INCLUDES
	${SRC}
    ${CORE_INCLUDES}
    ${LASLIB_INCLUDES})

set(APP_LIBRARIES 
    ${CORE_LIBRARIES}
	${LASLIB_LIBRARY}) 

add_executable(${PROJECT_NAME}
	${SOURCE_FILES}
	${HEADER_FILES}
	${RESOURCE_FILES})


include_directories(
    ${APP_INCLUDES})

target_link_libraries(${PROJECT_NAME}
	${APP_LIBRARIES})

add_custom_target(uninstall-${PROJECT_NAME}
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt)

