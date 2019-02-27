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
 * @file "modules/guidance_v_module/guidance_v_module.c"
 * @author Shuo Li
 * This module is a customized module, which receive thrust command from other module in float and transfer it to fix point number and send it to low level control
 */

#include "modules/guidance_v_module/guidance_v_module.h"
#include "modules/guidance_loop_controller/guidance_loop_controller.h"
#include "firmwares/rotorcraft/stabilization.h"

void guidance_v_module_init(void)
{

}

void guidance_v_module_enter(void)
{

}

#define FF_CMD_FRAC 18

void guidance_v_module_run(bool in_flight)
{
  int32_t inv_m;
  float m = 0.389;
  // m * g = guidance_v_nominal_throttle * MAX_PPRZ
  //guidance_v_nomimal_throttle = 0.4
  inv_m = BFP_OF_REAL(9.81 / (guidance_v_nominal_throttle * MAX_PPRZ), FF_CMD_FRAC); 
  float zdd = nn_cmd.thrust_ref/m;
  const int32_t g_m_zdd = (int32_t)BFP_OF_REAL(zdd, FF_CMD_FRAC);

  guidance_v_ff_cmd = g_m_zdd / inv_m;

  // guidance_v_ff_cmd = zdd /g * MAX_PPRZ*guidance_v_nomimal_throttle 
  
  /* feed forward command */
  guidance_v_ff_cmd = (guidance_v_ff_cmd << INT32_TRIG_FRAC) / guidance_v_thrust_coeff;
  Bound(guidance_v_ff_cmd, 0, 8640);

  /* our error feed back command                   */
  /* z-axis pointing down -> positive error means we need less thrust */
  guidance_v_fb_cmd = 0;

  guidance_v_delta_t = guidance_v_ff_cmd + guidance_v_fb_cmd;

  /* bound the result */
  Bound(guidance_v_delta_t, 0, MAX_PPRZ);
  printf("[guidance_v_module] altitude nn is running\n");
  stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
}
