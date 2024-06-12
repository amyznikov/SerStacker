CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

get_property(DSO_ENABLED GLOBAL PROPERTY BUILD_DSO)
if ( NOT  DSO_ENABLED ) 
   message(WARNING "DSO is not enabled. Skip dsol")
   return()
endif()

project(dsol CXX)

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

add_executable(${PROJECT_NAME}
	${SOURCE_FILES}
	${HEADER_FILES}
	${RESOURCE_FILES})

get_property(DSO_INCLUDES GLOBAL PROPERTY DSO_INCLUDE_DIRS)
target_include_directories( ${PROJECT_NAME} PUBLIC
    ${SRC}
    ${DSO_INCLUDES}
    ${CORE_INCLUDES})
    
target_link_libraries(${PROJECT_NAME} PUBLIC
	dso
    ${CORE_LIBRARIES})

add_custom_target(uninstall-${PROJECT_NAME}
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt)

