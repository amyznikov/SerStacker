cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(serstacker CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-march=native)
if( ENABLE_CALL_GRAPH_INSTRUMENTATION )
	add_definitions(-DENABLE_CALL_GRAPH_INSTRUMENTATION=1)
endif()

set (CMAKE_INCLUDE_CURRENT_DIR ON)
# set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_PREFIX_PATH "/usr/local") # need for some stupid ubuntus

if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv6l")
  set(APP_ARCH armv6)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7")
  set(APP_ARCH armv7)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64|armv8")
  set(APP_ARCH armv8)
elseif(CMAKE_SIZEOF_VOID_P EQUAL "8")
  set(APP_ARCH x86_64)
else()
  set(APP_ARCH i686)
endif ()

if(WIN32)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--stack,16000000")
endif()

message(STATUS "Detected architecture: ${APP_ARCH}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})


# ----------------------------------------------------------------------------
# OpenCV
set(OpenCV_MINIMUM_VERSION_REQUIRED "4.2")
find_package(OpenCV REQUIRED COMPONENTS core imgproc ximgproc imgcodecs features2d flann video videoio objdetect xobjdetect OPTIONAL_COMPONENTS xfeatures2d stereo) # freetype
if((OpenCV_VERSION VERSION_LESS ${OpenCV_MINIMUM_VERSION_REQUIRED}) )
   message(FATAL_ERROR "ERROR: OpenCV version >= ${OpenCV_MINIMUM_VERSION_REQUIRED} is required but '${OpenCV_VERSION}' is found")
   return()
endif()
message(STATUS "OpenCV:  ${OpenCV_VERSION}")
message(STATUS "xfeatures2d:  ${OpenCV_xfeatures2d_FOUND}")
if ( OpenCV_xfeatures2d_FOUND ) 
	add_definitions(-DHAVE_OpenCV_xfeatures2d=1)
endif()	
if ( OpenCV_stereo_FOUND ) 
	add_definitions(-DHAVE_OpenCV_stereo=1)
endif()	
set(enable_OpenCV TRUE)
message (STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
include_directories(${OpenCV_INCLUDE_DIRS})


# ----------------------------------------------------------------------------
# cfitsio
#
unset(CFITSIO_INCLUDE_DIRS CACHE)
unset(CFITSIO_LIBRARY CACHE)
find_path(CFITSIO_INCLUDE_DIRS fitsio2.h)
find_library(CFITSIO_LIBRARY cfitsio)

if( CFITSIO_INCLUDE_DIRS AND CFITSIO_LIBRARY ) 
    message(STATUS "cfitsio FOUND: CFITSIO_INCLUDE_DIRS=${CFITSIO_INCLUDE_DIRS} CFITSIO_LIBRARY=${CFITSIO_LIBRARY}")
    add_definitions(-DHAVE_CFITSIO=1)
	include_directories(${CFITSIO_INCLUDE_DIRS})
else()
	
    message(WARNING "Can not locate cfitsio library under search parth.
      Install please cfitsio package: 
     	ubuntu: libcfitsio-dev
     	archliux: cfitsio
     	windows: https://github.com/healpy/cfitsio")
     
	unset(CFITSIO_INCLUDE_DIRS CACHE)
	unset(CFITSIO_LIBRARY CACHE)
endif()

# ----------------------------------------------------------------------------
# libraw
#
unset(LIBRAW_INCLUDE_DIRS CACHE)
unset(LIBRAW_LIBRARY CACHE)
find_path(LIBRAW_INCLUDE_DIRS libraw/libraw.h)
find_library(LIBRAW_LIBRARY raw)

if( LIBRAW_INCLUDE_DIRS AND LIBRAW_LIBRARY ) 
    message(STATUS "libraw FOUND: LIBRAW_INCLUDE_DIRS=${LIBRAW_INCLUDE_DIRS} LIBRAW_LIBRARY=${LIBRAW_LIBRARY}")
    add_definitions(-DHAVE_LIBRAW=1)
	include_directories(${LIBRAW_INCLUDE_DIRS})    
else()
    message(WARNING "Can not locate libraw directory under search path.
      Install please libraw package (ubuntu: libraw-dev, archliux: libraw, windows: https://www.libraw.org/)")
	unset(LIBRAW_INCLUDE_DIRS CACHE)
	unset(LIBRAW_LIBRARY CACHE)
endif()


# ----------------------------------------------------------------------------
# openraw
#

foreach (v 0.3 0.2 0.1 )
  unset(OPENRAW_INCLUDE_DIRS CACHE)
  find_path(OPENRAW_INCLUDE_DIRS "libopenraw-${v}/libopenraw"
    /usr/local/include
    /opt/local/include
    /usr/include)
  if ( OPENRAW_INCLUDE_DIRS )
    set(OPENRAW_INCLUDE_DIRS "${OPENRAW_INCLUDE_DIRS}/libopenraw-${v}")
    break()
  endif() 
endforeach()
if ( NOT OPENRAW_INCLUDE_DIRS )
	set(OPENRAW_INCLUDE_DIRS)
    message(WARNING "Can not locate libopenraw-0.3 directory under search path.
      Install please libopenraw-0.3 package:
      	ubuntu: libopenraw-dev 
      	archliux: libopenraw
      	windows: https://libopenraw.freedesktop.org")
else()
	set(OPENRAW_LIBS openraw)
	add_definitions(-DHAVE_LIBOPENRAW=1)
	include_directories(${OPENRAW_INCLUDE_DIRS})
	message(STATUS "libopenraw: ${OPENRAW_INCLUDE_DIRS}")  
endif()


# ----------------------------------------------------------------------------
# TIFF : TIFF_LIBRARIES TIFF_INCLUDE_DIR
#
find_package(TIFF)
if ( NOT TIFF_FOUND )
  message(FATAL_ERROR "TIFF package is mandatory but is not found in your system. 
  	Install please libtiff package.")
else()
	include_directories(${TIFF_INCLUDE_DIR})
endif()



# ----------------------------------------------------------------------------
# FFMPEG
#
find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)
if( AVCODEC_INCLUDE_DIR AND AVCODEC_LIBRARY ) 
    message(STATUS "avcodec FOUND: AVCODEC_INCLUDE_DIR=${AVCODEC_INCLUDE_DIR} AVCODEC_LIBRARY=${AVCODEC_LIBRARY}")
    add_definitions(-DHAVE_AVCODEC=1)
else()
    message(FATAL_ERROR "Can not locate avcodec library under search path.
      Install please avcodec package (ubuntu: libavcodec-dev, archliux: ffmpeg, windows: http://www.ffmpeg.org)")
endif()

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)
if( AVFORMAT_INCLUDE_DIR AND AVFORMAT_LIBRARY ) 
    message(STATUS "avformat FOUND: AVFORMAT_INCLUDE_DIR=${AVFORMAT_INCLUDE_DIR} AVFORMAT_LIBRARY=${AVFORMAT_LIBRARY}")
    add_definitions(-DHAVE_AVFORMAT=1)
else()
    message(FATAL_ERROR "Can not locate avformat library under search path.
      Install please avformat package (ubuntu: libavformat-dev, archliux: ffmpeg, windows: http://www.ffmpeg.org)")
endif()

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)
if( AVUTIL_INCLUDE_DIR AND AVUTIL_LIBRARY ) 
    message(STATUS "avutil FOUND: AVUTIL_INCLUDE_DIR=${AVUTIL_INCLUDE_DIR} AVUTIL_LIBRARY=${_LIBRARY}")
    add_definitions(-DHAVE_AVUTIL=1)
else()
    message(FATAL_ERROR "Can not locate avutil library under search path.
      Install please avutil package (ubuntu: libavutil-dev, archliux: ffmpeg, windows: http://www.ffmpeg.org)")
endif()

find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)
if( SWSCALE_INCLUDE_DIR AND SWSCALE_LIBRARY ) 
    message(STATUS "swscale FOUND: SWSCALE_INCLUDE_DIR=${SWSCALE_INCLUDE_DIR} SWSCALE_LIBRARY=${SWSCALE_LIBRARY}")
    add_definitions(-DHAVE_SWSCALE=1)
else()
    message(FATAL_ERROR "Can not locate swscale library under search path.
      Install please swscale package (ubuntu: libswscale-dev, archliux: ffmpeg, windows: http://www.ffmpeg.org)")
endif()

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)
if( AVDEVICE_INCLUDE_DIR AND AVDEVICE_LIBRARY ) 
    message(STATUS "avdevice FOUND: AVDEVICE_INCLUDE_DIR=${AVDEVICE_INCLUDE_DIR} AVDEVICE_LIBRARY=${AVDEVICE_LIBRARY}")
    add_definitions(-DHAVE_AVDEVICE=1)
else()
    message(WARNING "Can not locate avdevice library  under search path.
      Install please avdevice package (ubuntu: libavdevice-dev, archliux: ffmpeg, windows: http://www.ffmpeg.org)")
	unset(AVDEVICE_INCLUDE_DIR CACHE)
	unset(AVDEVICE_LIBRARY CACHE)
endif()

set(FFMPEG_INCLUDE_DIRS ${SWSCALE_INCLUDE_DIR} ${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR} ${AVDEVICE_INCLUDE_DIR})
set(FFMPEG_LIBRARIES ${SWSCALE_LIBRARY} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY})
include_directories(${FFMPEG_INCLUDE_DIRS})


# ----------------------------------------------------------------------------
# Intel TBB
#
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




# ----------------------------------------------------------------------------
# libconfig
#
set(LIBCONFIG_MINIMUM_VERSION_REQUIRED "1.7")
find_package(libconfig QUIET)
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

include_directories(${LIBCONFIG_INCLUDE_DIRS})
message (STATUS "libconfig : ${LIBCONFIG_VERSION} ${LIBCONFIG_INCLUDE_DIRS} ${LIBCONFIG_LIBRARIES}")

#
## ----------------------------------------------------------------------------
## QGLViewer (optional, for glddm team)
##
#set(HAVE_QGLViewer FALSE)  
#unset (QGLViewer_FOUND)
#
#if ( FALSE )
#	find_package(QGLViewer QUIET)
#	if ( NOT QGLViewer_FOUND )
#	
#		set(QGLVIEWER_INCLUDE_DIRS "")
#		set(QGLVIEWER_LIBRARY "")
#	
#	    message(WARNING "If you are glddm team members then you pobably need 3D clouds browsing.\n \
#	    	QGLViewer >= 2.7 is required for glddm 3D clouds browsing. \n \
#	        To support this functionality install pleaae fresh version of QGLViewer from AUR or from sources: \n \
#	        \n \
#	         wget http://www.libqglviewer.com/src/libQGLViewer-2.7.1.tar.gz && \\ \n \
#	         tar -xzvf libQGLViewer-2.7.1.tar.gz && \\ \n \
#	         cd libQGLViewer-2.7.1/QGLViewer && \\ \n \
#	         qmake && \\ \n \
#	         make && \\ \n \
#	         sudo make install && \\ \n \
#	         make clean\ \n \
#	     ")
#	
#	else()
#		set(QT_VERSION 5)
#	
#		# OpenGL is required for QGLViewer
#		# This option should affect only OpenGL-dependent targets
#		# In case of multiple choices prefer vendor OPENGL_gl_LIBRARY by default
#		# (usually libOpenGL.so and libGLX.so).
#		if( ${CMAKE_VERSION} VERSION_GREATER "3.10" AND NOT OpenGL_GL_PREFERENCE )
#		    set(OpenGL_GL_PREFERENCE GLVND)
#		endif()
#		
#		find_package(OpenGL QUIET)
#		if ( NOT OpenGL_FOUND )
#	        message(WARNING "OpenGL package is not found, disabling QGLViewer")
#	        set(QGLVIEWER_INCLUDE_DIRS)
#	        set(QGLVIEWER_LIBRARY)
#	        set(OPENGL_INCLUDE_DIR)
#	        set(OPENGL_LIBRARY_DIR)
#	        set(OPENGL_LIBRARIES)
#	        set(QGLViewer_FOUND FALSE)
#	    else ()
#	        find_file(freeglut_h_found GL/freeglut.h  /usr/local/include /usr/include)
#	        if ( NOT freeglut_h_found ) 
#	            message(WARNING "OpenGL package is not found, disabling QGLViewer")
#	            set(QGLVIEWER_INCLUDE_DIRS)
#	            set(QGLVIEWER_LIBRARY)
#	            set(OPENGL_INCLUDE_DIR)
#	            set(OPENGL_LIBRARY_DIR)
#	            set(OPENGL_LIBRARIES)
#	            set(QGLViewer_FOUND FALSE)
#	        else()
#	            set(HAVE_QGLViewer TRUE)
#	            add_definitions(-DHAVE_QGLViewer=1)
#	
#	            # Fixing GLEW PATH on OSX
#	            if (APPLE)
#	                find_path( GLEW_INCLUDE_DIR GL/glew.h
#	                    /usr/local/include
#	                    /usr/local/Cellar/glew/2.1.0/include
#	                    /System/Library/Frameworks/GLEW.framework/Versions/A/Headers
#	                    ${OPENGL_LIBRARY_DIR}
#	                )
#	                find_library( GLEW_GLEW_LIBRARY GLEW
#	                    /usr/local/lib
#	                    /usr/openwin/lib
#	                    /usr/X11R6/lib
#	                    /usr/local/Cellar/glew/2.1.0/lib
#	                )
#	                set( GLEW_FOUND TRUE )
#	                if(NOT GLEW_GLEW_LIBRARY)
#	                    set(GLEW_GLEW_LIBRARY "-framework GLEW" CACHE STRING "GLEW library for OSX")
#	                    set(GLEW_cocoa_LIBRARY "-framework Cocoa" CACHE STRING "Cocoa framework for OSX")
#	                    set(GLEW_FOUND FALSE )
#	                endif()
#	            endif (APPLE)
#	        endif()
#	    endif()	    
#	endif()
#endif()
#

# ----------------------------------------------------------------------------
# Qt
#
#set(QT_VERSION 5)
#unset(QT_VERSION)
if ( NOT QT_VERSION )
    find_package(Qt6 COMPONENTS Core QUIET)
	if (NOT Qt6_FOUND)
	    set(QT_VERSION 5 CACHE STRING "Qt version")
	else()
	    set(QT_VERSION 6 CACHE STRING "Qt version")
	endif()
endif()

set(Qt_REQUIRED_COMPONENTS Core Gui Widgets Xml PrintSupport Multimedia MultimediaWidgets OpenGL )

if ( "${QT_VERSION}" STREQUAL "6" )
 set(Qt_REQUIRED_COMPONENTS ${Qt_REQUIRED_COMPONENTS} OpenGLWidgets)
endif()




find_package(Qt${QT_VERSION} COMPONENTS ${Qt_REQUIRED_COMPONENTS}) 
message( STATUS "Qt=Qt${QT_VERSION}")

foreach( module ${Qt_REQUIRED_COMPONENTS} )
	get_target_property(QtModuleIncludePath Qt${QT_VERSION}::${module} INTERFACE_INCLUDE_DIRECTORIES)
	include_directories(${QtModuleIncludePath})
endforeach()

if (WIN32)
set(Shlwapi_LIB Shlwapi)
endif()

set(CORE_LIBRARIES
	core
	${LIBCONFIG_LIBRARIES}
	${OpenCV_LIBS}
	${FFMPEG_LIBRARIES}
	${TIFF_LIBRARIES}
	${LIBRAW_LIBRARY}
	${CFITSIO_LIBRARY}
	${OPENRAW_LIBS}
    ${TBB_LIBRARIES}
    ${Shlwapi_LIB}
	pthread)



# Walk over subdirs
add_subdirectory(core)
add_subdirectory(gui)
add_subdirectory(apps)

# pseudo-target for building libraries only
add_custom_target(libs)
add_dependencies(libs core)
#add_dependencies(libs gui)

#message("XXX INCLUDE_DIRECTORIES:")
#get_property(dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#	message("XX ${dir}")  
#endforeach()


add_custom_target(uninstall
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt
)
