cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
	set(ToolchainFilePath "${CMAKE_CURRENT_LIST_DIR}/Dependencies/LFramework.CMake/stm32_gcc.cmake")
	if(EXISTS ${ToolchainFilePath})
		set(CMAKE_TOOLCHAIN_FILE "${ToolchainFilePath}" CACHE PATH "toolchain file" FORCE)
		message("Toolchain file found and set to ${ToolchainFilePath}")
	else()
		message(FATAL_ERROR "Unable to resolve '${ToolchainFilePath}' for setting toolchain file.")
	endif()
endif()

project(AT32F403A.HelloWorld C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(AT32F403A.HelloWorld)

add_library(CommonCode INTERFACE)

include(Dependencies/LFramework.CMake/stm32_gcc.cmake)
include(Dependencies/Mcu.Cmake/at32/F403A.cmake)

add_subdirectory(Application)

add_subdirectory(Dependencies/LFramework)
add_subdirectory(Dependencies/AT32F403A.Library)

target_link_libraries(CommonCode INTERFACE
    AT32F403A.Library
    GCC::Specs::NoSys
    GCC::Specs::Nano
    AT32::F403A
)

target_sources(CommonCode INTERFACE
${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/AT32F403A_407_Firmware_Library/libraries/cmsis/cm4/device_support/startup/gcc/startup_at32f403a_407.s
)


target_link_libraries(AT32F403A.HelloWorld PRIVATE
    CommonCode
    
)

target_compile_definitions(AT32F403A.HelloWorld
PRIVATE
    LF_TARGET_OS=LF_OS_FREERTOS
    AT32F403ACGU7
)

gcc_add_linker_script(AT32F403A.HelloWorld PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/AT32F403A_407_Firmware_Library/libraries/cmsis/cm4/device_support/startup/gcc/linker/AT32F403AxG_FLASH.ld")
