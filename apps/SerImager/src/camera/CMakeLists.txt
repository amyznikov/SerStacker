include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB SOURCES *.c *.cc *.cpp)
file(GLOB HEADERS *.h *.hpp)
target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${HEADERS})

add_subdirectory(zwo_asi)
add_subdirectory(v4l2)
add_subdirectory(ffmpeg)
