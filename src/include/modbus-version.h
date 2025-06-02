/*
 * Libmodbus for MCU - Version 1.0.0
 * Based on libmodbus 3.1.11
 *
 * Copyright © William Chen <william_engineer@outlook.com>
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * This file is part of Libmodbus for MCU, an adaptation of libmodbus
 * for MCU platforms with hardware abstraction and optimization.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 */

#ifndef MODBUS_VERSION_H
#define MODBUS_VERSION_H

/* The major version, (1, if %LIBMODBUS_FOR_MCU_VERSION is 1.2.3) */
#define LIBMODBUS_FOR_MCU_VERSION_MAJOR (1)

/* The minor version (2, if %LIBMODBUS_VERSION is 1.2.3) */
#define LIBMODBUS_FOR_MCU_VERSION_MINOR (0)

/* The micro version (3, if %LIBMODBUS_FOR_MCU_VERSION is 1.2.3) */
#define LIBMODBUS_FOR_MCU_VERSION_MICRO (0)

/* The full version, like 1.2.3 */
#define LIBMODBUS_FOR_MCU_VERSION 1.0.0

/* The full version, in string form (suited for string concatenation)
 */
#define LIBMODBUS_FOR_MCU_VERSION_STRING "1.0.0"

/* The major version, (1, if %LIBMODBUS_VERSION is 1.2.3) */
#define LIBMODBUS_VERSION_MAJOR (3)

/* The minor version (2, if %LIBMODBUS_VERSION is 1.2.3) */
#define LIBMODBUS_VERSION_MINOR (1)

/* The micro version (3, if %LIBMODBUS_VERSION is 1.2.3) */
#define LIBMODBUS_VERSION_MICRO (11)

/* The full version, like 1.2.3 */
#define LIBMODBUS_VERSION 3.1.11

/* The full version, in string form (suited for string concatenation)
 */
#define LIBMODBUS_VERSION_STRING "3.1.11"

/* Numerically encoded version, eg. v1.2.3 is 0x010203 */
#define LIBMODBUS_VERSION_HEX                                                                      \
    ((LIBMODBUS_VERSION_MAJOR << 16) | (LIBMODBUS_VERSION_MINOR << 8) |                            \
     (LIBMODBUS_VERSION_MICRO << 0))

/* Evaluates to True if the version is greater than @major, @minor and @micro
 */
#define LIBMODBUS_VERSION_CHECK(major, minor, micro)                                               \
    (LIBMODBUS_VERSION_MAJOR > (major) ||                                                          \
     (LIBMODBUS_VERSION_MAJOR == (major) && LIBMODBUS_VERSION_MINOR > (minor)) ||                  \
     (LIBMODBUS_VERSION_MAJOR == (major) && LIBMODBUS_VERSION_MINOR == (minor) &&                  \
      LIBMODBUS_VERSION_MICRO >= (micro)))

#endif /* MODBUS_VERSION_H */
