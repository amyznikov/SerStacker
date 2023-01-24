find_path(V4L2_INCLUDES linux/v4l2-controls.h)
find_library(V4L2_LIB v4l2)
if( NOT V4L2_INCLUDES OR NOT V4L2_LIB ) 
    message(WARNING "Can not locate v4l2 library under search parth. Install please v4l-utils developmet package ")
    return()
endif()


find_path(V4LCONVERT_INCLUDES libv4lconvert.h)
find_library(V4LCONVERT_LIB v4lconvert)

if( NOT V4LCONVERT_INCLUDES OR NOT V4LCONVERT_LIB ) 
    message(WARNING "Can not locate v4lconvert library under search parth. Install please v4l-utils developmet package ")
    return()
endif()

message(STATUS "v4l2: ${V4L2_INCLUDES} ${V4LCONVERT_INCLUDES};  ${V4LCONVERT_LIB} ${V4L2_LIB}")


file(GLOB_RECURSE SOURCES *.c *.cc *.cpp)
file(GLOB_RECURSE HEADERS *.h *.hpp)
file(GLOB QRC_FILES *.qrc )

target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${V4LCONVERT_LIB} ${V4L2_LIB})

