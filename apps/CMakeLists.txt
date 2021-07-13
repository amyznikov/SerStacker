cmake_minimum_required(VERSION 3.1)

file(GLOB subdirs RELATIVE 
	${CMAKE_CURRENT_SOURCE_DIR} 
	${CMAKE_CURRENT_SOURCE_DIR}/*)

foreach (subdir ${subdirs} )
  if ( IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} )
	if( EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/CMakeLists.txt )
		message(STATUS "Add application: ${subdir}")
		add_subdirectory(${subdir})
	endif()
  endif()
endforeach()


add_custom_target(uninstall-apps
	xargs rm -vf < ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt
)
