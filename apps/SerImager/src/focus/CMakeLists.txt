include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB CURRENT_SOURCES *.c *.cc .cpp)
file(GLOB CURRENT_HEADERS *.h *.hpp)
file(GLOB QRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.qrc )

foreach(QRC_FILE ${QRC_FILES})
	qt5_add_resources(CURRENT_RESOURCES ${QRC_FILE})
endforeach()

target_sources(${PROJECT_NAME} PRIVATE ${CURRENT_SOURCES} ${CURRENT_HEADERS})

add_subdirectory(indigo)
