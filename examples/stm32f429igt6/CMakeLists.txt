# ============================================================================== #
# Author: William Chen
# Email : william_engineer@outlook.com
# ------------------------------------------------------------------------------ #
# This file is licensed under the MIT License as stated below
#
# License
# -------
#
# Copyright (c) 2022-2032 William Chen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ============================================================================== #

# ============================================================================== #
# set workspace path and target architecture
# ============================================================================== #
set (USER_WORKSPACE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
get_filename_component(LIBMODBUS_ROOT "${USER_WORKSPACE_PATH}/../.." ABSOLUTE)
include("${USER_WORKSPACE_PATH}/cmake/tool-chains/cortex_m4f.cmake")

# ============================================================================== #
# Project Info
# ============================================================================== #
# set CMake minimum version
cmake_minimum_required(VERSION 3.20)

# set project name
get_filename_component(USER_PROJECT_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${USER_PROJECT_NAME} VERSION 0.0.1 LANGUAGES C CXX ASM)
message(STATUS "**** Building project: ${CMAKE_PROJECT_NAME}, Version: ${CMAKE_PROJECT_VERSION} ****")

# set linker script
file(GLOB USER_LINKER_SCRIPT "${USER_WORKSPACE_PATH}/cmake/linker/*.ld")
if(USER_LINKER_SCRIPT)
  list(GET USER_LINKER_SCRIPT 0 USER_LINKER_SCRIPT)
  message(STATUS "**** Linker script file: ${USER_LINKER_SCRIPT} ****")
else()
  message(FATAL_ERROR "**** Can NOT find linker script file in cmake/linker! ****")
endif()

# set startup file
file(GLOB USER_STARTUP_ASM "${USER_WORKSPACE_PATH}/cmake/startup/*.s")
if(USER_STARTUP_ASM)
  list(GET USER_STARTUP_ASM 0 USER_STARTUP_ASM)
  message(STATUS "**** Startup file: ${USER_STARTUP_ASM} ****")
else()
  message(FATAL_ERROR "**** Can NOT find startup file in cmake/startup! ****")
endif()

# include header files
include_directories(${USER_WORKSPACE_PATH}/Drivers/)
include_directories(${USER_WORKSPACE_PATH}/Drivers/BSP/ETHERNET)
include_directories(${USER_WORKSPACE_PATH}/Drivers/BSP/IIC)
include_directories(${USER_WORKSPACE_PATH}/Drivers/BSP/KEY)
include_directories(${USER_WORKSPACE_PATH}/Drivers/BSP/LCD)
include_directories(${USER_WORKSPACE_PATH}/Drivers/BSP/LED)
include_directories(${USER_WORKSPACE_PATH}/Drivers/BSP/PCF8574)
include_directories(${USER_WORKSPACE_PATH}/Drivers/BSP/SDRAM)
include_directories(${USER_WORKSPACE_PATH}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
include_directories(${USER_WORKSPACE_PATH}/Drivers/CMSIS/Include)
include_directories(${USER_WORKSPACE_PATH}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy)
include_directories(${USER_WORKSPACE_PATH}/Drivers/STM32F4xx_HAL_Driver/Inc)
include_directories(${USER_WORKSPACE_PATH}/Drivers/SYSTEM/delay)
include_directories(${USER_WORKSPACE_PATH}/Drivers/SYSTEM/sys)
include_directories(${USER_WORKSPACE_PATH}/Drivers/SYSTEM/usart)
include_directories(${USER_WORKSPACE_PATH}/Middlewares)
include_directories(${USER_WORKSPACE_PATH}/Middlewares/FreeRTOS/include)
include_directories(${USER_WORKSPACE_PATH}/Middlewares/FreeRTOS/portable/GCC/ARM_CM4F)
include_directories(${USER_WORKSPACE_PATH}/Middlewares/lwip/arch)
include_directories(${USER_WORKSPACE_PATH}/Middlewares/lwip)
include_directories(${USER_WORKSPACE_PATH}/Middlewares/lwip/src/include)
include_directories(${USER_WORKSPACE_PATH}/Middlewares/MALLOC)
include_directories(${USER_WORKSPACE_PATH}/Middlewares/USMART)
include_directories(${USER_WORKSPACE_PATH}/User)
include_directories(${LIBMODBUS_ROOT}/arch)
include_directories(${LIBMODBUS_ROOT}/src/include)
include_directories(${LIBMODBUS_ROOT}/tests)

# add source files
file(GLOB USER_SOURCE_LIST  "${USER_WORKSPACE_PATH}/Drivers/*.c"
                            "${USER_WORKSPACE_PATH}/Middlewares/FreeRTOS/portable/MemMang/heap_4.c"
                            "${USER_WORKSPACE_PATH}/Middlewares/FreeRTOS/portable/GCC/ARM_CM4F/*.c"
                            "${USER_WORKSPACE_PATH}/Middlewares/FreeRTOS/*.c"
                            "${USER_WORKSPACE_PATH}/Middlewares/lwip/*.c"
                            "${USER_WORKSPACE_PATH}/Middlewares/MALLOC/*.c"
                            "${USER_WORKSPACE_PATH}/Middlewares/USMART/*.c"
                            "${USER_WORKSPACE_PATH}/User/*.c")

set(LIBMODBUS_SOURCE_LIST
    "${LIBMODBUS_ROOT}/src/modbus.c"
    "${LIBMODBUS_ROOT}/src/modbus-data.c"
    "${LIBMODBUS_ROOT}/src/modbus-rtu.c"
    "${LIBMODBUS_ROOT}/src/modbus-tcp.c"
    "${LIBMODBUS_ROOT}/arch/ports/stm32f429/modbus-rtu-hal.c"
    "${LIBMODBUS_ROOT}/tests/modbus_test.c")

# ============================================================================== #
# Compiler Behavior Control
# ============================================================================== #
# enable concise output from Makefile builds
set(CMAKE_VERBOSE_MAKEFILE OFF)

# ============================================================================== #
# Macro Definition
# ============================================================================== #
# add macro
add_definitions(
  -DNDEBUG
  -DSTM32F429xx
  -DUSE_HAL_DRIVER
  -D__ARMGCC__
  -D__MCU__
  -DUSING_NET_STACK_LWIP
#  -DUSING_NON_RTOS # Not using RTOS
  -DUSING_OS_TYPE_FREERTOS # using FreeRTOS
#  -DUSING_OS_TYPE_THREADX # using Threadx
)

# ============================================================================== #
# Build Project
# ============================================================================== #
if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  add_definitions()
else()
  add_definitions(-DLOG_BACKEND=LOG_BACKEND_NONE)
endif()
message(STATUS "**** Build for ${CMAKE_BUILD_TYPE} ****")

# set executable output path
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/build/bin)

# set linker parameters
set(CMAKE_EXE_LINKER_FLAGS
   "${CMAKE_EXE_LINKER_FLAGS} \
   -T \"${USER_LINKER_SCRIPT}\" \
   -Wl,-Map=\"${PROJECT_SOURCE_DIR}/build/${USER_PROJECT_NAME}.map\" \
   -Wl,-static,--gc-sections,--print-memory-usage")

# search static library
if(EXISTS "${USER_WORKSPACE_PATH}/library")
  file(GLOB_RECURSE USER_STATIC_LIBRARY_LIST "${USER_WORKSPACE_PATH}/library/*.a")
else()
  set(USER_STATIC_LIBRARY_LIST "")
  message(STATUS "**** Not found library path ****")
endif()

# generate executable file
add_executable(${USER_PROJECT_NAME}
               ${USER_STARTUP_ASM}
               ${USER_SOURCE_LIST}
               ${LIBMODBUS_SOURCE_LIST})

# link static library to target
target_link_libraries(${USER_PROJECT_NAME} -Wl,--start-group)
if(USER_STATIC_LIBRARY_LIST)
  foreach(ITEM_IN_USER_STATIC_LIBRARY_LIST ${USER_STATIC_LIBRARY_LIST})
  target_link_libraries(${USER_PROJECT_NAME} ${ITEM_IN_USER_STATIC_LIBRARY_LIST})
  message(${ITEM_IN_USER_STATIC_LIBRARY_LIST})
  endforeach() 
endif()
# link with the math library (m) which provides mathematical functions like trigonometric functions, logarithms, etc
target_link_libraries(${USER_PROJECT_NAME} m) 
# link with the C standard library (c) which provides basic functions and data types in the C programming language
target_link_libraries(${USER_PROJECT_NAME} c)
# link with the GCC runtime library (gcc) which provides runtime support related to the GCC compiler, including language-specific features and internal functions
target_link_libraries(${USER_PROJECT_NAME} gcc)
# link with the nosys library (nosys). this library is used in bare-metal environments or systems without an operating system
target_link_libraries(${USER_PROJECT_NAME} nosys)
target_link_libraries(${USER_PROJECT_NAME} -Wl,--end-group)

# ============================================================================== #
# Post Build Operation
# ============================================================================== #
# Generate bin and hex files
add_custom_command(
  TARGET ${USER_PROJECT_NAME}
  POST_BUILD
  COMMAND ${OBJCOPY} -Obinary "${EXECUTABLE_OUTPUT_PATH}/${USER_PROJECT_NAME}" "${EXECUTABLE_OUTPUT_PATH}/${USER_PROJECT_NAME}.bin"
  COMMAND ${OBJCOPY} -Oihex "${EXECUTABLE_OUTPUT_PATH}/${USER_PROJECT_NAME}" "${EXECUTABLE_OUTPUT_PATH}/${USER_PROJECT_NAME}.hex"
  COMMENT "Building ${USER_PROJECT_NAME}.bin and ${USER_PROJECT_NAME}.hex"
  # Display sizes
  COMMAND ${SIZE} --format=berkeley "${EXECUTABLE_OUTPUT_PATH}/${USER_PROJECT_NAME}"
  COMMENT "Invoking: Cross ARM GNU Print Size"
)
