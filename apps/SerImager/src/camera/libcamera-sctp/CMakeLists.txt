# libcamera-sctp

if ( NOT SCTP_FOUND )
  message(STATUS "SCTP NOT found. libcamera-sctp will disabled")
  return()
endif()

#
## Look for the header file (usually in /usr/include/netinet/sctp.h) for the library file (libsctp.so)
#find_path(SCTP_INCLUDE_DIR NAMES netinet/sctp.h)
#find_library(SCTP_LIBRARY NAMES sctp)
#
## Handle standard arguments and set SCTP_FOUND
#include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(SCTP DEFAULT_MSG SCTP_LIBRARY SCTP_INCLUDE_DIR)
#
#if(SCTP_FOUND)
#    set(SCTP_LIBRARIES ${SCTP_LIBRARY})
#    set(SCTP_INCLUDE_DIRS ${SCTP_INCLUDE_DIR})
#    message(STATUS "SCTP found: ${SCTP_LIBRARIES}")
#	add_definitions(-DHAVE_SCTP=1)
#else()
#    message(STATUS "SCTP NOT found. libcamera-sctp will disabled")
#	add_definitions(-DHAVE_SCTP=0)
#    return()
#endif()


file(GLOB_RECURSE SOURCES *.c *.cc *.cpp)
file(GLOB_RECURSE HEADERS *.h *.hpp)
file(GLOB QRC_FILES *.qrc )

target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PRIVATE ${SCTP_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} PRIVATE ${SCTP_LIBRARIES})
