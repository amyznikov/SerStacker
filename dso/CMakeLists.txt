CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

set_property(GLOBAL PROPERTY BUILD_DSO FALSE)

find_package(Eigen3) # REQUIRED
if ( NOT Eigen3_FOUND ) 
	message(WARNING "Eigen3 Not found. Skip dso")
	return()
else()
  get_target_property(Eigen3_INCLUDES Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  set(Eigen3_LIBS Eigen3::Eigen)
  message(STATUS "Eigen3: ${Eigen3_INCLUDES} ${Eigen3_LIBS}")
endif()

find_package(Sophus) # REQUIRED
if ( NOT Sophus_FOUND ) 
	message(WARNING "Sophus Not found. Skip dso")
	return()
else()
  get_target_property(Sophus_INCLUDES Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
  get_target_property(Sophus_LIBS Sophus::Sophus INTERFACE_LINK_LIBRARIES)
  message(STATUS "Sophus: ${Sophus_INCLUDES} ${Sophus_LIBS}")
endif()

find_package(LibZip) 
if ( NOT LibZip_FOUND ) 
	message(WARNING "LibZip Not found")
else()
	get_target_property(LibZip_INCLUDES libzip::zip INTERFACE_INCLUDE_DIRECTORIES)
	# get_target_property(LibZip_LIBS libzip::zip INTERFACE_LINK_LIBRARIES)
	set(LibZip_LIBS libzip::zip)
	message(STATUS "LibZip: ${LibZip_INCLUDES} ${LibZip_LIBS}")
endif()


project(dso CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if( NOT CMAKE_CXX_STANDARD ) 
	set (CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  if(NOT CMAKE_TOOLCHAIN_FILE)
    # it _must_ go before any project in order to work
    set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation Directory")
  else()
    #Android: set output folder to ${CMAKE_BINARY_DIR}
    set(LIBRARY_OUTPUT_PATH_ROOT ${CMAKE_BINARY_DIR} CACHE PATH "root for library output, set this to change where android libs are compiled to" )
    # any cross-compiling
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
  endif()
endif()




# ----------------------------------------------------------------------------
# target sources

file(GLOB_RECURSE CURRENT_SOURCES *.c *.cc *.cpp)
file(GLOB_RECURSE CURRENT_HEADERS *.h *.hpp)

add_library(${PROJECT_NAME} STATIC  ${CURRENT_HEADERS} ${CURRENT_SOURCES})

if ( LibZip_FOUND ) 
target_compile_definitions(${PROJECT_NAME} PUBLIC
     HAS_ZIPLIB=1)
endif()

set(SRC ${CMAKE_CURRENT_SOURCE_DIR})

set(DSO_INCLUDES 
    ${Eigen3_INCLUDES}
    ${Sophus_INCLUDES}
    ${LibZip_INCLUDES})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${SRC}
    ${CORE_INCLUDES}
    ${DSO_INCLUDES})

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${CORE_LIBRARIES}
    ${Sophus_LIBS}
    ${Eigen3_LIBS}
    ${LibZip_LIBS})

set_property(GLOBAL PROPERTY BUILD_DSO TRUE)
set_property(GLOBAL PROPERTY DSO_INCLUDE_DIRS ${DSO_INCLUDES})


add_custom_target(uninstall-${PROJECT_NAME}
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt
)
