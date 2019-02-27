/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/stm32/subsystems/actuators/actuators_shared_arch.h
 *  STM32 PWM and dualPWM servos shared functions.
 */

#ifndef ACTUATORS_DSHOT_ARCH_H
#define ACTUATORS_DSHOT_ARCH_H

#include "std.h"

#include BOARD_CONFIG


#ifndef ACTUATORS_DSHOT_NB
#define ACTUATORS_DSHOT_NB 4
#endif

extern int32_t actuators_dshot_values[ACTUATORS_DSHOT_NB];

extern void actuators_dshot_commit(void);

#define ActuatorDshotSet(_i, _v) { actuators_dshot_values[_i] = _v; }
#define ActuatorsDshotCommit  actuators_dshot_commit


#endif /* ACTUATORS_DSHOT_H */
