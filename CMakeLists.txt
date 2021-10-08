cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(serstacker CXX)

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

foreach (v 0.3 0.2 0.1 )
  unset(OPENRAW_INCLUDES CACHE)
  find_path(OPENRAW_INCLUDES "libopenraw-${v}/libopenraw"
    /usr/local/include
    /opt/local/include
    /usr/include)
  if ( OPENRAW_INCLUDES )
    set(OPENRAW_INCLUDES "${OPENRAW_INCLUDES}/libopenraw-${v}")
    break()
  endif() 
endforeach()
if ( NOT OPENRAW_INCLUDES )
  message(WARNING "Can not locate libopenraw-0.3 directory under search path.
    Install please libopenraw-0.3 package (ubuntu: libopenraw-dev, archliux libopenraw: windows: https://libopenraw.freedesktop.org)")
else()
  set(OPENRAW_LIBS openraw)
  add_definitions(-DHAVE_LIBOPENRAW=1)
  message(STATUS "libopenraw: ${OPENRAW_INCLUDES}")
endif()



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
# find_package(TBB)
# if ( ${TBB_FOUND} )
#   add_definitions(-DHAVE_TBB=1 -DTBB_SUPPRESS_DEPRECATED_MESSAGES=1)
#   set(TBB_LIBRARIES "tbb")    
# endif()
find_package(TBB QUIET)
if (TBB_FOUND)

  get_target_property(TBB_INCLUDES TBB::tbb INTERFACE_INCLUDE_DIRECTORIES)
  if ( NOT TBB_INCLUDES ) # This happend under SUSE: TBB_FOUND is TRUE but INTERFACE_INCLUDE_DIRECTORIES is not set
        SET(TBB_INCLUDES /usr/include)
        MESSAGE(WARNING, "INTERFACE_INCLUDE_DIRECTORIES was not set by TBBConfig.cmake, force TBB_INCLUDES to ${TBB_INCLUDES}")
  endif()
  
  set(TBB_LIBRARIES TBB::tbb)

elseif(UNIX)
  # It seems we are working on old version of ubuntu or apple, try to locate files manually

  find_path(TBB_INCLUDES tbb/tbb.h
    /usr/local/include/
    /opt/local/include/
    /usr/include)

  if ( NOT TBB_INCLUDES )
    message(FATAL_ERROR "Can not locate TBB include directory under search path. 
      Install please correct tbb package (ubuntu: libtbb-dev, archliux tbb: windows: https://github.com/oneapi-src/oneTBB.git)")
  endif()

  find_library(TBB_LIBRARIES
    NAMES tbb 
    PATHS /usr/local/lib
          /opt/local/lib
          /usr/lib
          ENV LD_LIBRARY_PATH
          ENV LIBRARY_PATH)

  if ( NOT TBB_LIBRARIES )
    message(FATAL_ERROR "Can not locate TBB library under search path. 
      Install please correct tbb package (ubuntu: libtbb-dev, archliux tbb: windows: https://github.com/oneapi-src/oneTBB.git)")
  endif()

  set(TBB_FOUND TRUE)
  
else()
    message(FATAL_ERROR "Can not locate TBB library under search path. 
      Install please correct tbb package (ubuntu: libtbb-dev, archliux tbb: windows: https://github.com/oneapi-src/oneTBB.git)") 
endif()

add_definitions(-DHAVE_TBB=1 -DTBB_SUPPRESS_DEPRECATED_MESSAGES=1)
message (STATUS "TBB: ${TBB_INCLUDES} ${TBB_LIBRARIES}")




# libconfig
set(LIBCONFIG_MINIMUM_VERSION_REQUIRED "1.7")
find_package(libconfig)
message(STATUS "libconfig [ ${LIBCONFIG_VERSION}, ${libconfig_VERSION}] found=${libconfig_FOUND}")
if ( NOT ${libconfig_FOUND} )
    message(FATAL_ERROR "libconfig version >= ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} is not found\n \
        Please install fresh version of libconfig manually from github: https://github.com/hyperrealm/libconfig.git")
endif()
if ( libconfig_VERSION AND NOT LIBCONFIG_VERSION ) # Th case of the lellers vary between ubuntu and arch linux
    set(LIBCONFIG_VERSION ${libconfig_VERSION})
endif()
if ( NOT LIBCONFIG_VERSION )
    message(FATAL_ERROR "Can not determine libconfig version, seems libconfig cmake skript is broken.
        Please install fresh version of libconfig manually from github: https://github.com/hyperrealm/libconfig.git")
endif()
if ( LIBCONFIG_VERSION VERSION_LESS ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} )
    message(FATAL_ERROR "libconfig version >= ${LIBCONFIG_MINIMUM_VERSION_REQUIRED} is required but libconfig version ${LIBCONFIG_VERSION} found.
        Please install fresh version of libconfig manually from github: https://github.com/hyperrealm/libconfig.git")
endif()

if ( NOT LIBCONFIG_INCLUDE_DIRS )
  if (WIN32)
    get_target_property(LIBCONFIG_INCLUDE_DIRS libconfig::libconfig INTERFACE_INCLUDE_DIRECTORIES)
  else()
    get_target_property(LIBCONFIG_INCLUDE_DIRS libconfig::config INTERFACE_INCLUDE_DIRECTORIES)
  endif()
endif()

if ( NOT LIBCONFIG_LIBRARIES ) 
  if (WIN32)
    set(LIBCONFIG_LIBRARIES libconfig::libconfig)
  else() 
    set(LIBCONFIG_LIBRARIES libconfig::config)
  endif()
endif()

message (STATUS "libconfig : ${LIBCONFIG_VERSION} ${LIBCONFIG_INCLUDE_DIRS} ${LIBCONFIG_LIBRARIES}")



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
