/*
 * Copyright (C) Shuo Li
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/guidance_v_module/guidance_v_module.h"
 * @author Shuo Li
 * This module is a customized module, which receive thrust command from other module in float and transfer it to fix point number and send it to low level control
 */

#ifndef GUIDANCE_V_MODULE_H
#define GUIDANCE_V_MODULE_H
#include "modules/guidance_h_module/guidance_h_module.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
 
extern void guidance_v_module_enter(void);

extern void guidance_v_module_run(bool in_flight);

extern void guidance_v_module_init(void);
#endif

