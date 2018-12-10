/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include "std.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"
#include "modules/guidance_loop_controller/guidance_loop_controller.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "modules/nn/nn.h"
#include "subsystems/actuators/motor_mixing.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(
      file_logger,
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz\n"
    );
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();

  fprintf(file_logger, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f ,%f,%f,%f,%f,%f,%d,%d,%d,%f,"
          "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", counter,
stateGetPositionNed_f()->x,
stateGetPositionNed_f()->y,
stateGetPositionNed_f()->z,
stateGetSpeedNed_f()->x,
stateGetSpeedNed_f()->y,
stateGetSpeedNed_f()->z,
stateGetNedToBodyEulers_f()->phi,
stateGetNedToBodyEulers_f()->theta,
stateGetNedToBodyEulers_f()->psi,
stateGetBodyRates_f()->p,
stateGetBodyRates_f()->q,
stateGetBodyRates_f()->r,
nn_cmd.thrust_ref,
nn_cmd.rate_ref,
controllerInUse,

pos_NWU.x,
pos_NWU.y,
pos_NWU.z,

vel_NWU.x,
vel_NWU.y,
vel_NWU.z,
guidance_v_delta_t,
nn_time,
psi_c,

rateRef.p_ref,
rateRef.q_ref,
rateRef.r_ref,
guidance_v_delta_t,
guidance_v_nominal_throttle,
sp_accel.z,

stateGetAccelNed_f()->x,
stateGetAccelNed_f()->y,
stateGetAccelNed_f()->z,

  debug_indi.z_sp,
  debug_indi.z_ref,
  debug_indi.z_error,
  debug_indi.vz_sp,
  debug_indi.az_sp,
  
imu.accel.x,
imu.accel.y,
imu.accel.z,

weight_nn,
motor_mixing.commands[0],
motor_mixing.commands[1],
motor_mixing.commands[2],
motor_mixing.commands[3],

motor_cmd.trim[0],
motor_cmd.trim[1],
motor_cmd.trim[2],
motor_cmd.trim[3],

motor_cmd.thrust[0],
motor_cmd.thrust[1],
motor_cmd.thrust[2],
motor_cmd.thrust[3],

motor_cmd.pitch[0],
motor_cmd.pitch[1],
motor_cmd.pitch[2],
motor_cmd.pitch[3]
         );
  counter++;
}
