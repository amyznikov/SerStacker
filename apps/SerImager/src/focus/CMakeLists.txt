include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB CURRENT_SOURCES *.c *.cc .cpp)
file(GLOB CURRENT_HEADERS *.h *.hpp)
#file(GLOB QRC_FILES *.qrc )
#
#foreach(QRC_FILE ${QRC_FILES})
#	if ( "${QT_VERSION}" STREQUAL "6" )
#		qt6_add_resources(CURRENT_SOURCES ${QRC_FILE})
#	else()
#		qt5_add_resources(CURRENT_SOURCES ${QRC_FILE})
#	endif()
#endforeach()

target_sources(${PROJECT_NAME} PRIVATE ${CURRENT_SOURCES} ${CURRENT_HEADERS})

add_subdirectory(indigo)
