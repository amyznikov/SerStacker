CMAKE_MINIMUM_REQUIRED(VERSION 3.16)


find_package(Eigen3) # REQUIRED
if ( NOT Eigen3_FOUND ) 
	message(WARNING "Eigen3 Not found. skip dsol")
	return()
endif()

get_target_property(Eigen3_INCLUDES Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
#get_target_property(Eigen3_LIBS Eigen3::Eigen INTERFACE_LINK_LIBRARIES)
set(Eigen3_LIBS Eigen3::Eigen)
message(STATUS "Eigen3: ${Eigen3_INCLUDES} ${Eigen3_LIBS}")


find_package(Sophus) # REQUIRED
if ( NOT Sophus_FOUND ) 
	message(WARNING "Sophus Not found. skip dsol")
	return()
endif()
get_target_property(Sophus_INCLUDES Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(Sophus_LIBS Sophus::Sophus INTERFACE_LINK_LIBRARIES)
message(STATUS "Sophus: ${Sophus_INCLUDES} ${Sophus_LIBS}")


find_package(LibZip) # REQUIRED
if ( NOT LibZip_FOUND ) 
	message(WARNING "LibZip Not found")
else()
	get_target_property(LibZip_INCLUDES libzip::zip INTERFACE_INCLUDE_DIRECTORIES)
	# get_target_property(LibZip_LIBS libzip::zip INTERFACE_LINK_LIBRARIES)
	set(LibZip_LIBS libzip::zip)
	message(STATUS "LibZip: ${LibZip_INCLUDES} ${LibZip_LIBS}")
endif()


project(dsol CXX)

if( NOT CMAKE_CXX_STANDARD )
	set (CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
set(CMAKE_LINK_WHAT_YOU_USE OFF)


set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)

file(GLOB_RECURSE SOURCE_FILES
	${SRC}/*.c
	${SRC}/*.cc
	${SRC}/*.cpp)

file(GLOB_RECURSE HEADER_FILES
	${SRC}/*.h
	${SRC}/*.hpp)

add_executable(${PROJECT_NAME}
	${SOURCE_FILES}
	${HEADER_FILES}
	${RESOURCE_FILES})


target_include_directories( ${PROJECT_NAME} PUBLIC
    ${SRC}
    ${CORE_INCLUDES})
    
target_link_libraries(${PROJECT_NAME} PUBLIC
	dso
    ${CORE_LIBRARIES})

add_custom_target(uninstall-${PROJECT_NAME}
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt)

