CMAKE_MINIMUM_REQUIRED(VERSION 3.16)
project(qgltest CXX)

# Disabe this app
return()

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


file(GLOB_RECURSE CURRENT_SOURCES src/*.c src/*.cc src/*.cpp )
file(GLOB_RECURSE CURRENT_HEADERS src/*.h src/*.hpp )
file(GLOB_RECURSE QRC_FILES src/*.qrc )

foreach(QRC_FILE ${QRC_FILES})
	if ( "${QT_VERSION}" STREQUAL "6" )
		qt6_add_resources(CURRENT_SOURCES ${QRC_FILE})
	else()
		qt5_add_resources(CURRENT_SOURCES ${QRC_FILE})
	endif()
endforeach()

add_executable(${PROJECT_NAME} ${CURRENT_SOURCES} ${CURRENT_HEADERS})


set(LIBRARIES  
  gui
  Qt${QT_VERSION}::Core
  Qt${QT_VERSION}::Gui
  Qt${QT_VERSION}::Widgets
  Qt${QT_VERSION}::Multimedia
  Qt${QT_VERSION}::MultimediaWidgets 
  Qt${QT_VERSION}::Xml
  Qt${QT_VERSION}::PrintSupport
  GLU
  GL)

if ( "${QT_VERSION}" STREQUAL "6" )
set(LIBRARIES
  ${LIBRARIES}
  Qt${QT_VERSION}::OpenGLWidgets) 
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${LIBRARIES}
  ${CORE_LIBRARIES})



# Install


set(MY_RUNTIME_PATH bin)
set(MY_DESKTOP_ENTRY_PATH ${CMAKE_INSTALL_PREFIX}/share/applications)
set(MY_DESKTOP_ENTRY_COMMENT "${PROJECT_NAME}")
set(MY_APP_ICONS_PATH ${CMAKE_INSTALL_PREFIX}/share/app-install/icons)

if( EXISTS src/app.desktop.in)
	configure_file(src/app.desktop.in
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.desktop
		@ONLY)
endif()

if(EXISTS src/icons/app-icon.png)
	configure_file(src/icons/app-icon.png
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

