# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

option(FEATURE_DEV "Development stuff (emulated eFuse, etc)" ON)
option(FEATURE_DEBUG "Debug build stuff" ON)
option(FEATURE_PROD "Debug prod stuff" OFF)
option(FEATURE_EFUSE_EMU "Emulate efuse in ROM. Relevant for PROD" OFF)


set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig.prj")
set(SDKCONFIG_DEFAULTS "${CMAKE_SOURCE_DIR}/sdkconfig.defaults")


set(BOARDS_ROOT ${CMAKE_SOURCE_DIR}/boards)

set(BOARD esp32s3)
set(IDF_TARGET esp32s3)


if(FEATURE_PROD)
	if(${FEATURE_DEBUG} OR ${FEATURE_DEV})
		message(WARNING "PROD build with debug options FEATURE_DEBUG=${FEATURE_DEBUG} FEATURE_DEV=${FEATURE_DEV}")
	endif()
	set(SDKCONFIG_DEFAULTS "${SDKCONFIG_DEFAULTS};${CMAKE_SOURCE_DIR}/sdkconfig.prd")
	add_compile_definitions(FEATURE_PROD)
endif()

if(FEATURE_DEBUG)
	message(WARNING "FEATURE_DEBUG is set")
	set(SDKCONFIG_DEFAULTS "${SDKCONFIG_DEFAULTS};${CMAKE_SOURCE_DIR}/sdkconfig.dbg")
	add_compile_definitions(FEATURE_DEBUG)
	set(FEATURE_DEV ON)
endif()

if(FEATURE_DEV)
	message(WARNING "FEATURE_DEV is set")
	set(SDKCONFIG_DEFAULTS "${SDKCONFIG_DEFAULTS};${CMAKE_SOURCE_DIR}/sdkconfig.dev")
	add_compile_definitions(FEATURE_DEV)
	set(FEATURE_EFUSE_EMU ON)
endif()

if(FEATURE_EFUSE_EMU)
	message(WARNING "FEATURE_EFUSE_EMU is set")
	set(SDKCONFIG_DEFAULTS "${SDKCONFIG_DEFAULTS};${CMAKE_SOURCE_DIR}/sdkconfig.efu")
	add_compile_definitions(FEATURE_EFUSE_EMU)
endif()

set(SDKCONFIG_DEFAULTS "${SDKCONFIG_DEFAULTS};${BOARDS_ROOT}/${BOARD}/sdkconfig")


add_compile_options(-w) # Turn off warnings until esp_tinyusb is updated IEC-86
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(ti700bridge)
