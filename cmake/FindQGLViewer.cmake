FIND_PACKAGE(Qt5 COMPONENTS Core Xml OpenGL Gui Widgets)
IF(NOT Qt5_FOUND)
  MESSAGE("Qt5 not found. Install it and set Qt5_DIR accordingly")
  IF (WIN32)
    MESSAGE("  In Windows, Qt5_DIR should be something like C:/Qt/5.4/msvc2013_64_opengl/lib/cmake/Qt5")
  ENDIF()
ENDIF()


unset(QGLVIEWER_INCLUDE_DIRS)
unset(QGLVIEWER_LIBRARY)
unset(QGLVIEWER_LIBRARY_RELEASE)
unset(QGLVIEWER_LIBRARY_DEBUG)

FIND_PATH(QGLVIEWER_INCLUDE_DIRS QGLViewer/qglviewer.h
    /usr/include/
    /opt/local/include/
    /usr/local/include/
    /sw/include/QGLViewer
    ${QGLVIEWER_PATH}/include/
    ENV QGLVIEWERROOT
  )

find_library(QGLVIEWER_LIBRARY_RELEASE
  NAMES qglviewer QGLViewer qglviewer-qt5 QGLViewer-qt5
  PATHS /usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
        ${QGLVIEWER_PATH}/lib/
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH
  PATH_SUFFIXES QGLViewer QGLViewer/release
)

find_library(QGLVIEWER_LIBRARY_DEBUG
  NAMES dqglviewer dQGLViewer dqglviewer-qt5 dQGLViewer-qt5 QGLViewerd2
  PATHS /usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
        ${QGLVIEWER_PATH}/lib/
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH
  PATH_SUFFIXES QGLViewer QGLViewer/debug
)

if(QGLVIEWER_LIBRARY_RELEASE)
  if(QGLVIEWER_LIBRARY_DEBUG)
    set(QGLVIEWER_LIBRARY optimized ${QGLVIEWER_LIBRARY_RELEASE} debug ${QGLVIEWER_LIBRARY_DEBUG})
  else()
    set(QGLVIEWER_LIBRARY ${QGLVIEWER_LIBRARY_RELEASE})
  endif()
endif()

if( QGLVIEWER_INCLUDE_DIRS AND (QGLVIEWER_LIBRARY_RELEASE OR QGLVIEWER_LIBRARY_DEBUG) )

	include(FindPackageHandleStandardArgs)
	find_package_handle_standard_args(QGLViewer DEFAULT_MSG
	  QGLVIEWER_INCLUDE_DIRS QGLVIEWER_LIBRARY)
	
	get_filename_component(QGLVIEWER_LIBRARYDIR ${QGLVIEWER_LIBRARY} DIRECTORY)
	
	set(QGLViewer_FOUND TRUE)
else()

	set(QGLViewer_FOUND FALSE)
endif()
