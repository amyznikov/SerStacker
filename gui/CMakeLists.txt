CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

project(gui CXX)

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




# ----------------------------------------------------------------------------
# target sources

file(GLOB_RECURSE SRC_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
 
file(GLOB_RECURSE HEADER_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

file(GLOB_RECURSE QRC_FILES
	"${CMAKE_CURRENT_SOURCE_DIR}/*.qrc" )


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    ${QGLVIEWER_INCLUDE_DIRS} 
    ${CORE_INCLUDES})


# AUTOMOC problem is that it merges all mocs into single file
# Single automoc file prevents later dependency-only linking.
# It is why I schedule the moc-ing manually here.
set(CMAKE_AUTOMOC OFF)
foreach(_header ${HEADER_FILES})

	file(STRINGS "${_header}" _strings REGEX "Q_OBJECT")

	if( NOT "${_strings}" STREQUAL "" )
    	file(RELATIVE_PATH _relpath ${CMAKE_CURRENT_SOURCE_DIR} "${_header}")
		set(_mocfile "${CMAKE_CURRENT_BINARY_DIR}/${_relpath}.moc.cc")
		qt5_generate_moc("${_header}" "${_mocfile}")
    	list(APPEND SRC_FILES "${_mocfile}")

		# include(AddFileDependencies)
    	# add_file_dependencies("${_impl}" "${_mocfile}")
    endif()

endforeach()


foreach(QRC_FILE ${QRC_FILES})
	qt5_add_resources(RESOURCE_FILES 
		${QRC_FILE})
endforeach()

message("RESOURCE_FILES =${RESOURCE_FILES}")

add_library(${PROJECT_NAME} STATIC 
	${HEADER_FILES} 
	${SRC_FILES} 
	${RESOURCE_FILES})


add_custom_target(uninstall-${PROJECT_NAME}
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt
)
