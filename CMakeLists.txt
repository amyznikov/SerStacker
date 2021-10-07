cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)


# OpenCV
set(OpenCV_MINIMUM_VERSION_REQUIRED "3.4")
set(OpenCV_FRESH_CV4_API_REQUIRED "4.0.1")
find_package(OpenCV REQUIRED)
if((OpenCV_VERSION VERSION_GREATER ${OpenCV_MINIMUM_VERSION_REQUIRED}) OR (OpenCV_VERSION VERSION_EQUAL ${OpenCV_MINIMUM_VERSION_REQUIRED}))
   if((OpenCV_VERSION VERSION_GREATER ${OpenCV_FRESH_CV4_API_REQUIRED}) OR (OpenCV_VERSION VERSION_EQUAL ${OpenCV_FRESH_CV4_API_REQUIRED}))
       message("Setting usage of fresh API, as OpenCV version exceeds ${OpenCV_FRESH_CV4_API_REQUIRED}")
       set(USE_FRESH_CV4_API ON)
   endif()
else()
   message(FATAL_ERROR "ERROR: OpenCV version >= ${OpenCV_MINIMUM_VERSION_REQUIRED} is required but '${OpenCV_VERSION}' is found")
   return()
endif()

#find_package(Eigen3 QUIET)
#find_package(Ceres QUIET)

# libraw
# find_package(libraw REQUIRED)
set(LIBRAW_LIBS raw)

# openraw
# find_package(openraw REQUIRED)
set(OPENRAW_LIBS openraw)
set(OPENRAW_INCLUDES /usr/include/libopenraw-0.3)

# cfitsio
# find_package(cfitsio REQUIRED)
set(CFITSIO_LIBS cfitsio)

# TIFF : TIFF_LIBRARIES TIFF_INCLUDE_DIR
find_package(TIFF)
if ( NOT TIFF_FOUND )
  message( FATAL_ERROR "TIFF package is mandatory but is not found in your system. Install libtiff package please.")
endif()



# FFMPEG
find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)
find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)
find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)
find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)
find_path(AVDEVICE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)
set(FFMPEG_INCLUDE_DIRS ${SWSCALE_INCLUDE_DIR} ${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR} ${AVDEVICE_INCLUDE_DIR})
set(FFMPEG_LIBRARIES ${SWSCALE_LIBRARY} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY})


# Intel TBB
find_package(TBB)
if ( ${TBB_FOUND} )
  add_definitions(-DHAVE_TBB=1 -DTBB_SUPPRESS_DEPRECATED_MESSAGES=1)
  set(TBB_LIBRARIES "tbb")    
endif()



# libconfig
set(LIBCONFIG_MINIMUM_VERSION_REQUIRED "1.7")
find_package(libconfig)
if( LIBCONFIG_FOUND AND (LIBCONFIG_VERSION VERSION_GREATER ${LIBCONFIG_MINIMUM_VERSION_REQUIRED}) OR (LIBCONFIG_VERSION VERSION_EQUAL ${LIBCONFIG_MINIMUM_VERSION_REQUIRED}))
   message(STATUS "libconfig ${LIBCONFIG_VERSION} found")
   add_definitions(-DHAVE_LIBCONFIG=1)

   if(APPLE)
     SET (LIBCONFIG_INCLUDE_DIRS /usr/local/include/)
     SET (LIBCONFIG_LIBRARIES /usr/local/lib/libconfig.dylib)
     INCLUDE_DIRECTORIES(${LIBCONFIG_INCLUDE_DIRS})
   endif(APPLE)

else()
   message("WARNING: libconfig ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} NOT FOUND, libConfig related functionality will unavailable")
endif()


# Qt5
set(Qt5_MINIMUM_VERSION_REQUIRED "5.6")
foreach ( module Qt5Core Qt5Gui Qt5Widgets Qt5Xml Qt5PrintSupport Qt5Multimedia Qt5MultimediaWidgets)
    find_package(${module} ${Qt5_MINIMUM_VERSION_REQUIRED})
    if ( NOT ${module}_FOUND )
        message( FATAL_ERROR "${module} ${Qt5_MINIMUM_VERSION_REQUIRED} NOT FOUND")
    endif()
endforeach()

set(CORE_INCLUDES
	${CMAKE_CURRENT_SOURCE_DIR}
	${LIBCONFIG_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${FFMPEG_INCLUDE_DIRS}
	${OPENRAW_INCLUDES}
	${TIFF_INCLUDE_DIR})

set(CORE_LIBRARIES
	core
	${LIBCONFIG_LIBRARIES}
	${OpenCV_LIBS}
	${FFMPEG_LIBRARIES}
	${TIFF_LIBRARIES}
	${LIBRAW_LIBS}
	${CFITSIO_LIBS}
	${OPENRAW_LIBS}
    ${TBB_LIBRARIES}
	pthread)



# Walk over subdirs
add_subdirectory(core)
add_subdirectory(gui)
add_subdirectory(apps)

# pseudo-target for building libraries only
add_custom_target(libs)
add_dependencies(libs core)
#add_dependencies(libs gui)


add_custom_target(uninstall
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt
)
