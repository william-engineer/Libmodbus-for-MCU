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

# compiler tool chain settings
cmake_host_system_information(RESULT USER_HOST_OS_INFO QUERY OS_NAME)
if(${USER_HOST_OS_INFO} STREQUAL "Windows")
  set(CMAKE_C_COMPILER "arm-none-eabi-gcc.exe")
  set(CMAKE_CXX_COMPILER "arm-none-eabi-g++.exe")
  set(AS "arm-none-eabi-as.exe")
  set(AR "arm-none-eabi-ar.exe")
  set(OBJCOPY "arm-none-eabi-objcopy.exe")
  set(OBJDUMP "arm-none-eabi-objdump.exe")
  set(SIZE "arm-none-eabi-size.exe")
else()
  set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
  set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
  set(AS "arm-none-eabi-as")
  set(AR "arm-none-eabi-ar")
  set(OBJCOPY "arm-none-eabi-objcopy")
  set(OBJDUMP "arm-none-eabi-objdump")
  set(SIZE "arm-none-eabi-size")
endif()

# set c/c++ programming language standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
enable_language(ASM)

# export complie commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# enable color diagnostics throughout
set(CMAKE_COLOR_DIAGNOSTICS ON)

# adjust the default behaviour of the find commands:
# search headers and libraries in the target environment
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
# search programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# this makes the test compiles use static library option 
# so that we don't need to pre-set linker flags and scripts
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# gcc include path
find_program(USER_GCC_TOOLCHAIN_DIR ${CMAKE_C_COMPILER})
if(USER_GCC_TOOLCHAIN_DIR)
  get_filename_component(USER_GCC_TOOLCHAIN_DIR ${USER_GCC_TOOLCHAIN_DIR} DIRECTORY)
else()
  message(FATAL_ERROR "**** Can NOT find C compiler path! ****")
endif()
set(SYSTEM_PATH "-isystem ${USER_GCC_TOOLCHAIN_DIR}/../arm-none-eabi/include")

# set compiler flags 
# Note that DO NOT use --specs=rdimon.specs at the same time with --specs=nosys.specs
set(USER_COMMON_FLAGS "${MCPU_FLAGS} \
                       ${VFP_FLAGS} \
                       ${SYSTEM_PATH} \
                       --specs=nano.specs \
                       --specs=nosys.specs \
                       -Wall -fmessage-length=0 -ffunction-sections -fdata-sections")

# set release flags base on USER_COMMON_FLAGS
set(CMAKE_C_FLAGS_RELEASE "-O3 ${USER_COMMON_FLAGS} -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${USER_COMMON_FLAGS} -fno-exceptions -DNDEBUG")
set(CMAKE_ASM_FLAGS_RELEASE "${MCPU_FLAGS} ${VFP_FLAGS} -x assembler-with-cpp")

# set debug flags base on USER_COMMON_FLAGS
set(CMAKE_C_FLAGS_DEBUG "-O0 -g ${USER_COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g ${USER_COMMON_FLAGS} -fno-exceptions")
set(CMAKE_ASM_FLAGS_DEBUG "${MCPU_FLAGS} ${VFP_FLAGS} -x assembler-with-cpp")

if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(STATUS "**** Maximum optimization for speed ****")
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  message(STATUS "**** No optimization, debug info included ****")
else()
  message(STATUS "**** Unkonw build type: ${CMAKE_BUILD_TYPE} ****")
endif()