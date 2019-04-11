/*
 * Copyright (C) MAVLab
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
 * @file "modules/ctrl/dronerace//dronerace.c"
 * @author MAVLab
 * Autonomous Drone Race
 */


#include "modules/ctrl/dronerace/dronerace.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "subsystems/abi.h"
#include "state.h"
#include "filter.h"
#include "control.h"
#include "ransac.h"
#include "flightplan.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/autopilot_guided.h"

// to know if we are simulating:
#include "generated/airframe.h"

float dt = 1.0f / 512.f;

float input_phi;
float input_theta;
float input_psi;

volatile int input_cnt = 0;
volatile float input_dx = 0;
volatile float input_dy = 0;
volatile float input_dz = 0;

uint8_t previous_autopilot_mode;

#include <stdio.h>
// sending the divergence message to the ground station:
static void send_dronerace(struct transport_tx *trans, struct link_device *dev)
{
  float fx = dr_state.time;
  float fy = dr_state.x; // Flows
  float fz = dr_state.y;

  float cx = dr_state.vx; // Covariance
  float cy = dr_state.vy;
  float cz = 0;

  float gx = dr_vision.dx; // Vision
  float gy = dr_vision.dy;
  float gz = dr_vision.dz;

  float ex = dr_vision.cnt;
  float ey = 0;
  float ez = POS_FLOAT_OF_BFP(guidance_v_z_sp);

  int32_t ix = dr_state.assigned_gate_index; // Error
  float iy = dr_ransac.buf_size; // Error
  float iz = 0;

  pprz_msg_send_OPTICAL_FLOW_HOVER(trans, dev, AC_ID, &fx, &fy, &fz,
                                   &cx, &cy, &cz,
                                   &gx, &gy, &gz,
                                   &ex, &ey, &ez,
                                   &ix, &iy, &iz);
}


// ABI receive gates!
#ifndef DRONE_RACE_ABI_ID
// Receive from: 33 (=onboard vision) 34 (=jevois) or 255=any
#define DRONE_RACE_ABI_ID ABI_BROADCAST
#endif

static abi_event gate_detected_ev;


static void gate_detected_cb(uint8_t sender_id __attribute__((unused)), int32_t cnt, float dx, float dy, float dz, float vx __attribute__((unused)), float vy __attribute__((unused)), float vz __attribute__((unused)))
{
  // Logging
  input_cnt = cnt;
  input_dx = dx;
  input_dy = dy;
  input_dz = dz;

}

void dronerace_init(void)
{
  // Receive vision
  AbiBindMsgRELATIVE_LOCALIZATION(DRONE_RACE_ABI_ID, &gate_detected_ev, gate_detected_cb);

  // Send telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW_HOVER, send_dronerace);

  dronerace_enter();
  filter_reset();
}

float psi0 = 0;

void dronerace_enter(void)
{
  psi0 = stateGetNedToBodyEulers_f()->psi;
  dr_state.psi = psi0;
  //filter_reset();
  prediction_correct(); // it is called by module_enter(), when the mode is changed, the prediction should go to estimation instead of 0
  control_reset();
  previous_autopilot_mode = autopilot.mode;
}

#ifndef PREDICTION_BIAS_PHI
#define PREDICTION_BIAS_PHI        0.0f
#endif

#ifndef PREDICTION_BIAS_THETA
#define PREDICTION_BIAS_THETA      0.0f
#endif

void dronerace_periodic(void)
{
    if(previous_autopilot_mode != autopilot.mode)
    {
        control_reset();
    }


  //  test_guidance_indi_temp_run();
  input_phi = stateGetNedToBodyEulers_f()->phi;
  input_theta = stateGetNedToBodyEulers_f()->theta;
  input_psi = stateGetNedToBodyEulers_f()->psi - psi0;

  filter_predict(input_phi,input_theta,input_psi, dt);

  // Vision update
  // printf("input count, vision count: %d, %d\n", input_cnt, dr_vision.cnt);
  if (input_cnt > dr_vision.cnt) {
    dr_vision.cnt = input_cnt;
    dr_vision.dx = input_dx;
    dr_vision.dy = input_dy;
    dr_vision.dz = input_dz;

    filter_correct();
  }


  previous_autopilot_mode = autopilot.mode;
}

void dronerace_set_rc(UNUSED float rt, UNUSED float rx, UNUSED float ry, UNUSED float rz)
{
}

void dronerace_get_cmd(float* alt, float* phi, float* theta, float* psi_cmd)
{

  control_run();

  *phi = dr_control.phi_cmd;
  *theta = dr_control.theta_cmd;
  *psi_cmd = dr_control.psi_cmd + psi0;
  *alt = - dr_control.z_cmd;

  guidance_v_z_sp = POS_BFP_OF_REAL(dr_control.z_cmd);
  
}


