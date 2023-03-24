# file(GLOB asi_imager_SRCS *.cpp)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set(ASI_SDK_DIR ASI_Windows_SDK_V1.27)
    set(asi_libfilename "ASICamera2.dll")
else()
    set(ASI_SDK_DIR ASI_linux_mac_SDK_V1.27)
    set(asi_libfilename "libASICamera2.a")
endif()

set(ASI_ARCH_x86_64 x64)
set(ASI_ARCH_i686 x86)
set(ASI_ARCH_armv8 armv8)
set(ASI_ARCH_armv7 armv7)
set(ASI_ARCH_armv6 armv6)

set(ASI_TARGET_ARCH ${ASI_ARCH_${APP_ARCH}})
if( "${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin")
    set(ASI_TARGET_ARCH "mac")
endif()

include_directories(${ASI_SDK_DIR}/include)
set(asi_driver_library ${CMAKE_CURRENT_SOURCE_DIR}/${ASI_SDK_DIR}/lib/${ASI_TARGET_ARCH}/${asi_libfilename})

file(GLOB SOURCES *.c *.cc *.cpp)
file(GLOB HEADERS *.h *.hpp)
file(GLOB QRC_FILES *.qrc )

target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${HEADERS})

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux" OR ${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
  target_link_libraries(${PROJECT_NAME} PRIVATE ${asi_driver_library} -lusb-1.0)
endif()
if(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
	target_link_libraries(${PROJECT_NAME} ${asi_driver_library} "-framework CoreFoundation" "-framework IOKit" -lobjc)
endif()

# Use a different name than INDI, to avoid package conflicts
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	install(FILES ${ASI_SDK_DIR}/lib/asi.rules DESTINATION ${UDEVRULES_INSTALL_DIR} RENAME 97-asi.rules) 
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
  install(FILES ${asi_driver_library} DESTINATION .)
endif()

