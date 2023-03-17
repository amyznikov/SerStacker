file(GLOB_RECURSE SOURCES *.c *.cc *.cpp)
file(GLOB_RECURSE HEADERS *.h *.hpp)
file(GLOB QRC_FILES *.qrc )

target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${V4LCONVERT_LIB} ${V4L2_LIB})

