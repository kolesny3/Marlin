/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#define  X_IS_TRINAMIC ( defined( X_DRIVER_TYPE) )
#define  Y_IS_TRINAMIC ( defined( Y_DRIVER_TYPE) )
#define  Z_IS_TRINAMIC ( defined( Z_DRIVER_TYPE) )
#define X2_IS_TRINAMIC ( defined(X2_DRIVER_TYPE) )
#define Y2_IS_TRINAMIC ( defined(Y2_DRIVER_TYPE) )
#define Z2_IS_TRINAMIC ( defined(Z2_DRIVER_TYPE) )
#define E0_IS_TRINAMIC ( defined(E0_DRIVER_TYPE) )
#define E1_IS_TRINAMIC ( defined(E1_DRIVER_TYPE) )
#define E2_IS_TRINAMIC ( defined(E2_DRIVER_TYPE) )
#define E3_IS_TRINAMIC ( defined(E3_DRIVER_TYPE) )
#define E4_IS_TRINAMIC ( defined(E4_DRIVER_TYPE) )

#define  X_IS_TMC(MODEL) (  X_DRIVER_TYPE == MODEL )
#define  Y_IS_TMC(MODEL) (  Y_DRIVER_TYPE == MODEL )
#define  Z_IS_TMC(MODEL) (  Z_DRIVER_TYPE == MODEL )
#define X2_IS_TMC(MODEL) ( X2_DRIVER_TYPE == MODEL )
#define Y2_IS_TMC(MODEL) ( Y2_DRIVER_TYPE == MODEL )
#define Z2_IS_TMC(MODEL) ( Z2_DRIVER_TYPE == MODEL )
#define E0_IS_TMC(MODEL) ( E0_DRIVER_TYPE == MODEL )
#define E1_IS_TMC(MODEL) ( E1_DRIVER_TYPE == MODEL )
#define E2_IS_TMC(MODEL) ( E2_DRIVER_TYPE == MODEL )
#define E3_IS_TMC(MODEL) ( E3_DRIVER_TYPE == MODEL )
#define E4_IS_TMC(MODEL) ( E4_DRIVER_TYPE == MODEL )

#define HAVE_TMC(MODEL) ( X_IS_TMC(MODEL) || Y_IS_TMC(MODEL) || Z_IS_TMC(MODEL) \
                      || X2_IS_TMC(MODEL) || Y2_IS_TMC(MODEL) || Z2_IS_TMC(MODEL) \
                      || E0_IS_TMC(MODEL) || E1_IS_TMC(MODEL) || E2_IS_TMC(MODEL) || E3_IS_TMC(MODEL) || E4_IS_TMC(MODEL))
#define HAS_TRINAMIC (X_IS_TRINAMIC || Y_IS_TRINAMIC || Z_IS_TRINAMIC \
                  || X2_IS_TRINAMIC || Y2_IS_TRINAMIC || Z2_IS_TRINAMIC \
                  || E0_IS_TRINAMIC || E1_IS_TRINAMIC || E2_IS_TRINAMIC || E3_IS_TRINAMIC || E4_IS_TRINAMIC)
