include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB CURRENT_SOURCES *.c *.cc *.cpp)
file(GLOB CURRENT_HEADERS *.h *.hpp)
target_sources(${PROJECT_NAME} PRIVATE ${CURRENT_SOURCES} ${CURRENT_HEADERS})

add_subdirectory(zwo_asi)
add_subdirectory(v4l2)
