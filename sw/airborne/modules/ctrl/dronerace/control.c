
#include "std.h"


#include "control.h"
#include "filter.h"
#include "flightplan.h"
#include "ransac.h"
#include <math.h>
#include "firmwares/rotorcraft/autopilot_guided.h"
#include "state.h"

// Variables
struct dronerace_control_struct dr_control;
struct pid_term_struct pid_term = {0.0,0.0,0.0};
struct indi_controller_struct indi_ctrl;

float k_p_vel_x = 0.3;
float k_d_vel_x = 0.1;
float k_p_vel_y = 0.3;
float k_d_vel_y = 0.1;

float k_p_pos_x = 0.8;
float k_d_pos_x = 0.1;
float k_p_pos_y = 0.8;
float k_d_pos_y = 0.1;
float vx_error_previous = 0.0;
float vy_error_previous = 0.0;
int flag_record_video = 0;


// Settings


// Slow speed
#define CTRL_MAX_SPEED  2.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(35)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(25)    // rad
#define CTRL_MAX_R      RadOfDeg(225)    // rad/sec

void control_reset(void)
{
  // Reset flight plan logic
  flightplan_reset();

  // Reset own variables
  dr_control.psi_ref = 0;
  dr_control.psi_cmd = 0;
  vx_error_previous = 0.0;
  vy_error_previous = 0.0;
  indi_ctrl.previous_x_err = 0.0;
  indi_ctrl.previous_y_err = 0.0;
  indi_ctrl.previous_vx_err = 0.0;
  indi_ctrl.previous_vy_err = 0.0;
}


void control_run(void)
{
  float dt = 1.0/512.0;
  float psi, vxcmd, vycmd, r_cmd, ax, ay;
  // Propagate the flightplan
  flightplan_run();
  psi = dr_state.psi;

  // Heading controller
  r_cmd = 2.0*(dr_fp.psi_set - dr_control.psi_ref);

  // Apply rate limit
  Bound(r_cmd, -CTRL_MAX_R, CTRL_MAX_R);
  dr_control.psi_ref += r_cmd * dt;


  if(gates[dr_fp.gate_nr].psi == RadOfDeg(0.0) || gates[dr_fp.gate_nr].psi == RadOfDeg(180))
  {
    vxcmd = (dr_fp.x_set - filteredX) * k_p_pos_x - filteredVx * D_FORWARD;
    vycmd = (dr_fp.y_set - filteredY) * k_p_pos_y - filteredVy * D_LATERAL;
  }
  else
  {
    vxcmd = (dr_fp.x_set - filteredX) * k_p_pos_x - filteredVx * D_LATERAL;
    vycmd = (dr_fp.y_set - filteredY) * k_p_pos_y - filteredVy * D_FORWARD;
  }

  if(!waypoints_dr[dr_fp.gate_nr].brake) {
      vxcmd += 10.0f * cosf(waypoints_dr[dr_fp.gate_nr].psi);
      vycmd += 10.0f * sinf(waypoints_dr[dr_fp.gate_nr].psi);
  }

  if(dr_ransac.buf_size < 5)
  {
	  vxcmd = 0.0;
	  vycmd = 0.0;
  }


  Bound(vxcmd, -CTRL_MAX_SPEED, CTRL_MAX_SPEED);
  Bound(vycmd, -CTRL_MAX_SPEED, CTRL_MAX_SPEED);

  /*------------- for log--------------------- */
  pid_term.vx_cmd = vxcmd;
  pid_term.vy_cmd = vycmd;
  indi_ctrl.vx_cmd = vxcmd;
  indi_ctrl.vy_cmd = vycmd;
  /*------------------------------------------ */

  // Speed to Attitude
  float vx_err = vxcmd-filteredVx;
  float vy_err = vycmd-filteredVy;
  float dv_x = (vx_err-vx_error_previous)*512.0;
  float dv_y = (vy_err-vy_error_previous)*512.0;

  ax = vx_err * k_p_vel_x + dv_x * k_d_vel_x + vxcmd * RadOfDeg(10.0f) / 3.0f;
  ay = vy_err * k_p_vel_y + dv_y * k_d_vel_y + vycmd * RadOfDeg(10.0f) / 3.0f;

  Bound(ax, -CTRL_MAX_PITCH, CTRL_MAX_PITCH);
  Bound(ay, -CTRL_MAX_PITCH, CTRL_MAX_PITCH);

  dr_control.phi_cmd   = - sinf(psi) * ax + cosf(psi) * ay;
  dr_control.theta_cmd = - cosf(psi) * ax - sinf(psi) * ay;
  dr_control.psi_cmd   = dr_fp.psi_set;
  dr_control.z_cmd   = dr_fp.z_set;
  vx_error_previous = vx_err;
  vy_error_previous = vy_err;
}


