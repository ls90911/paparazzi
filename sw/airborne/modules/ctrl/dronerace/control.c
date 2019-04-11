
#include "std.h"


#include "control.h"
#include "filter.h"
#include "flightplan.h"
#include "ransac.h"
#include <math.h>
#include "firmwares/rotorcraft/autopilot_guided.h"

// Variables
struct dronerace_control_struct dr_control;
struct pid_term_struct pid_term = {0.0,0.0,0.0};
struct indi_controller_struct indi_ctrl;

float k_p_vel_x = 0.5;
float k_d_vel_x = 0.1;
float k_p_vel_y = 0.5;
float k_d_vel_y = 0.1;

float k_p_pos_x = 0.5;
float k_d_pos_x = 0.0;
float k_p_pos_y = 0.5;
float k_d_pos_y = 0.0;
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

static float angle180(float r)
{
  if (r < RadOfDeg(-180))
  {
    r += RadOfDeg(360.0f);
  }
  else if (r > RadOfDeg(180.0f))
  {
    r -= RadOfDeg(360.0f);
  }

  return r;
}


void control_run(void)
{
  float psi, vxcmd, vycmd, r_cmd, ax, ay;
  float dt = 1.0/512.0;
  // Propagate the flightplan
  flightplan_run();

  // Variables
  psi = dr_state.psi;

  // Heading controller
  r_cmd = 2.0*(dr_fp.psi_set - dr_control.psi_ref);

  // Find shortest turn
  //r_cmd = angle180(r_cmd);

  // Apply rate limit
  Bound(r_cmd, -CTRL_MAX_R, CTRL_MAX_R);
  dr_control.psi_ref += r_cmd * dt;

  //printf("wp = (%f,%f)\n",dr_fp.x_set,dr_fp.y_set);
  autopilot_guided_goto_ned(dr_fp.x_set,dr_fp.y_set,-1.5,dr_control.psi_ref);

}


