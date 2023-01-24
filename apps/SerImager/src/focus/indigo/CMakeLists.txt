
unset(INDIGO_INCLUDE_DIRS CACHE)
unset(INDIGO_LIBRARY CACHE)
find_path(INDIGO_INCLUDE_DIRS indigo/indigo_bus.h)
find_library(INDIGO_LIBRARY indigo)

if( NOT INDIGO_INCLUDE_DIRS OR NOT INDIGO_LIBRARY ) 
  message(WARNING "Can not locate indigo library under search parth.
     Install please indigo package: 
     github: https://github.com/indigo-astronomy/indigo")
     
  unset(INDIGO_INCLUDE_DIRS CACHE)
  unset(INDIGO_LIBRARY CACHE)
	
  return()
endif()

message(STATUS "indigo: INDIGO_INCLUDE_DIRS=${INDIGO_INCLUDE_DIRS} INDIGO_LIBRARY=${INDIGO_LIBRARY}")

target_compile_definitions(${PROJECT_NAME} PRIVATE -DHAVE_INDIGO=1)

if( "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin" )
  target_compile_definitions(${PROJECT_NAME} PRIVATE -DINDIGO_MACOS=1)
elseif( "${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
  target_compile_definitions(${PROJECT_NAME} PRIVATE -DINDIGO_LINUX=1)
else()
endif()


file(GLOB_RECURSE CURRENT_SOURCES *.c *.cc *.cpp)
file(GLOB_RECURSE CURRENT_HEADERS *.h *.hpp)
#file(GLOB_RECURSE QRC_FILES *.qrc)
#
#foreach(QRC_FILE ${QRC_FILES})
#	if ( "${QT_VERSION}" STREQUAL "6" )
#		message("XXXXX qt6_add_resources(${QRC_FILE}")
#		qt6_add_resources(CURRENT_SOURCES ${QRC_FILE})
#	else()
#		message("XXXXX qt5_add_resources(${QRC_FILE}")
#		qt5_add_resources(CURRENT_SOURCES ${QRC_FILE})
#	endif()
#endforeach()


target_sources(${PROJECT_NAME} PRIVATE ${CURRENT_SOURCES} ${CURRENT_HEADERS} )
target_link_libraries(${PROJECT_NAME} PRIVATE ${INDIGO_LIBRARY})
