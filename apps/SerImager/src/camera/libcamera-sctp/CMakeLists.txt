# libcamera
#set(LIBCAMERA_LINK_LIBRARIES)
#set(LIBCAMERA_INCLUDE_DIRS)
#
#find_package(PkgConfig QUIET) # REQUIRED
#if ( NOT PkgConfig_FOUND )
#	message( WARNING "PkgConfig NOT FOUND, libcamera can not be detected.")
#else()
#	pkg_check_modules(LIBCAMERA QUIET libcamera)
#	if ( NOT LIBCAMERA_VERSION ) 
#		message(WARNING "libcamera nor found. Install please libcamera package 
#			https://aur.archlinux.org/packages/libcamera-git 
#			https://libcamera.org")
#         return()
#    else()
#		message(STATUS "libcamera: ${LIBCAMERA_VERSION} ${LIBCAMERA_INCLUDE_DIRS} ${LIBCAMERA_LINK_LIBRARIES}")
#		add_definitions(-DHAVE_LIBCAMERA=1)
#    endif()
#endif()
#

file(GLOB_RECURSE SOURCES *.c *.cc *.cpp)
file(GLOB_RECURSE HEADERS *.h *.hpp)
file(GLOB QRC_FILES *.qrc )

#target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${HEADERS})
#target_include_directories(${PROJECT_NAME} PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBCAMERA_LINK_LIBRARIES} -lsctp)

target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PRIVATE )
target_link_libraries(${PROJECT_NAME} PRIVATE -lsctp)

