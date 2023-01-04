CMAKE_MINIMUM_REQUIRED(VERSION 3.20)
project(SerImager CXX)

if( NOT CMAKE_CXX_STANDARD )
	set (CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

# cmake_policy(SET CMP0079 NEW)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
set(CMAKE_LINK_WHAT_YOU_USE OFF)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)


set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(PROJECT_INCLUDES 
  ${SRC} 
  ${CORE_INCLUDES} 
  Qt5::Widgets
  Qt5::OpenGL
  Qt5::Xml
  Qt5::PrintSupport)

include_directories(${PROJECT_INCLUDES})

set(CURRENT_LINK_LIBRARIES 
  gui
  Qt5::Widgets
  Qt5::OpenGL
  Qt5::Multimedia
  Qt5::MultimediaWidgets 
  Qt5::Xml
  Qt5::PrintSupport
  ${OPENGL_LIBRARIES}
  ${CORE_LIBRARIES})

file(GLOB CURRENT_SOURCES
	${SRC}/*.c
	${SRC}/*.cc
	${SRC}/*.cpp
	)

file(GLOB CURRENT_HEADERS
	${SRC}/*.h
	${SRC}/*.hpp
	)

file(GLOB_RECURSE QRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.qrc" )
foreach(QRC_FILE ${QRC_FILES})
	qt5_add_resources(CURRENT_RESOURCES ${QRC_FILE})
endforeach()

add_executable(${PROJECT_NAME}
	${CURRENT_SOURCES}
	${CURRENT_HEADERS}
	${CURRENT_RESOURCES})

set(UDEVRULES_INSTALL_DIR "/lib/udev/rules.d" CACHE STRING "Base directory for udev rules")
set(FIRMWARE_INSTALL_BASEDIR "/lib/firmware/" CACHE STRING "Base directory for firmware files")

include_directories(${SRC}/camera)
add_subdirectory(${SRC}/camera)
include_directories(${SRC}/focus)
add_subdirectory(${SRC}/focus)



target_link_libraries(${PROJECT_NAME} PRIVATE
	${CURRENT_LINK_LIBRARIES})



# Install

set(MY_RUNTIME_PATH bin)
set(MY_DESKTOP_ENTRY_PATH ${CMAKE_INSTALL_PREFIX}/share/applications)
set(MY_DESKTOP_ENTRY_COMMENT "${PROJECT_NAME}")
set(MY_APP_ICONS_PATH ${CMAKE_INSTALL_PREFIX}/share/app-install/icons)

if( EXISTS ${SRC}/app.desktop.in)
	configure_file(${SRC}/app.desktop.in
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.desktop
		@ONLY)
endif()

if(EXISTS ${SRC}/icons/app-icon.png)
	configure_file(${SRC}/icons/app-icon.png
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.png
		COPYONLY)
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME
	DESTINATION ${MY_RUNTIME_PATH})

if( EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.desktop )
	install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.desktop
		DESTINATION ${MY_DESKTOP_ENTRY_PATH})
endif()

if( EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.png )
	install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.png
		DESTINATION ${MY_APP_ICONS_PATH})
endif()

add_custom_target(uninstall-${PROJECT_NAME}
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt)

