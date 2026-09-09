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

# set target architecture operating system name, if no operating system is used, set to Generic
set(CMAKE_SYSTEM_NAME Generic)

# set target architecture processor name
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)

# set RTOS information if used
# set(RTOS_NAME "FreeRTOS")
# set(RTOS_ARCH "cortex_m3")
# set(RTOS_TOOLCHAIN "gnu")

add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)


set(MCPU_FLAGS "-mthumb -mcpu=cortex-m4")
set(VFP_FLAGS "-mfloat-abi=hard -mfpu=fpv4-sp-d16")

MESSAGE(STATUS "**** platform: ${MCPU_FLAGS} ${VFP_FLAGS} ${FLOAT_ABI} ****")

INCLUDE(${CMAKE_CURRENT_LIST_DIR}/arm_none_eabi.cmake)