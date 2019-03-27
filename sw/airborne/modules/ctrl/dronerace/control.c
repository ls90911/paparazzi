
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
struct pid_term_struct pid_term = {0.0,0.0,0.0,0.0,0.0,0.0};
struct reference_generator_struct ref;
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
float x_error_previous = 0.0;
float y_error_previous = 0.0;
float vx_cmd = 0.0;
float vy_cmd = 0.0;



// Settings


// Slow speed
#define CTRL_MAX_SPEED  2.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(35)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(25)    // rad
#define CTRL_MAX_R      RadOfDeg(225)    // rad/sec

/*
// Max speed for bebop
#define CTRL_MAX_SPEED  6.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(35)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(35)    // rad
#define CTRL_MAX_R      RadOfDeg(45)    // rad/sec
*/

/*
// Race drone
#define CTRL_MAX_SPEED  10              // m/s
#define CTRL_MAX_PITCH  RadOfDeg(45)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(45)    // rad
#define CTRL_MAX_R      RadOfDeg(180)   // rad/sec
*/


control_cnt = 0;
void control_reset(void)
{
  // Reset flight plan logic
  flightplan_reset();
  reset_reference();
  filter_reset();
  control_cnt = 0;

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

//static float angle180(float r)
//{
//  if (r < RadOfDeg(-180))
//  {
//    r += RadOfDeg(360.0f);
//  }
//  else if (r > RadOfDeg(180.0f))
//  {
//    r -= RadOfDeg(360.0f);
//  }
//
//  return r;
//}


void control_run(void)
{
  control_cnt++;
  float r_cmd;
  float dt = 1.0/512.0;
  float psi = stateGetNedToBodyEulers_f()->psi;
  
  // Propagate the flightplan
  flightplan_run();
  //update_reference_run();

  // Heading controller
  r_cmd = 2.0*(dr_fp.psi_set - dr_control.psi_ref);


  // Apply rate limit
  Bound(r_cmd, -CTRL_MAX_R, CTRL_MAX_R);
  dr_control.psi_ref += r_cmd * dt;



  float x_error = dr_fp.x_set - filteredX;
  float y_error = dr_fp.y_set - filteredY;
  vx_cmd = k_p_pos_x*x_error + k_d_pos_x * (x_error - x_error_previous)/512.0;
  vy_cmd = k_p_pos_y*y_error + k_d_pos_y * (y_error - y_error_previous)/512.0;

  float vx_error = vx_cmd - filteredVx;
  float vy_error = vy_cmd - filteredVy;
  float ax_cmd = k_p_vel_x * (vx_error) + k_d_vel_x * (vx_error - vx_error_previous)/512.0;
  float ay_cmd = k_p_vel_y * (vy_error) + k_d_vel_y * (vy_error - vy_error_previous)/512.0;

  dr_control.phi_cmd   = - sinf(psi) * ax_cmd + cosf(psi) * ay_cmd;
  dr_control.theta_cmd = - cosf(psi) * ax_cmd - sinf(psi) * ay_cmd;
  dr_control.psi_cmd   = dr_control.psi_ref;
  dr_control.z_cmd   = dr_fp.z_set;



  //autopilot_guided_goto_ned(dr_fp.x_set,dr_fp.y_set,-1.5,dr_control.psi_ref);
  //----------------------------------------------------------
  //  test trachcan with optitrack
  if(control_cnt / 512.0 < 50)
  {
    autopilot_guided_goto_ned(0.0,0.0,-1.0,0);
  }
  else if(control_cnt / 512.0 < 100)
  {
    autopilot_guided_goto_ned(3.0,0.0,-1.0,3.14/2);
  }
  else if(control_cnt / 512.0 < 15)
  {
    autopilot_guided_goto_ned(3.0,3.0,-1.0,3.14);
  }
  else if(control_cnt / 512.0 <20) 
  {
    autopilot_guided_goto_ned(0.0,3.0,-1.0,3.14);
  }
  else if(control_cnt / 512.0 <25)
  {
	  control_cnt = 0;
  
  }
  //----------------------------------------------------------
}

void reference_init()
{
    ref.k_p_x_local = K_P_X_LOCAL;
    ref.k_v_x_local = K_V_X_LOCAL;
    ref.k_p_y_local = K_P_Y_LOCAL;
    ref.k_v_y_local = K_V_Y_LOCAL;
}

void reset_reference()
{
    ref.pos.x = filteredX;
    ref.pos.y = filteredY;
    ref.vel.x = filteredVx;
    ref.vel.y = filteredVy;
    reset_local_reference();
}

void reset_local_reference()
{
    float deltaX = filteredX- waypoints_dr[dr_fp.gate_nr].x;
    float deltaY = filteredY- waypoints_dr[dr_fp.gate_nr].y;
    float psi = waypoints_dr[dr_fp.gate_nr].psi;
    ref.pos_local.x= cos(psi)*deltaX+sin(psi)*deltaY;
    ref.pos_local.y= -sin(psi)*deltaX+cos(psi)*deltaY;
}

void update_reference_run()
{
    // intergrating in local frame
   float a_x_local =  -ref.pos_local.x*ref.k_p_x_local - ref.k_v_x_local * ref.vel_local.x;
   ref.pos_local.x += 1.0/512*ref.vel_local.x;
   ref.vel_local.x += 1.0/512*a_x_local;

   float a_y_local =  -ref.pos_local.y*ref.k_p_y_local - ref.k_v_y_local * ref.vel_local.y;
   ref.pos_local.y += 1.0/512*ref.vel_local.y;
   ref.vel_local.y += 1.0/512*a_y_local;

   // transform to global frame
   //float psi = waypoints_dr[dr_fp.gate_nr].psi;
   ref.pos.x = waypoints_dr[dr_fp.gate_nr].x;
   ref.pos.y = waypoints_dr[dr_fp.gate_nr].y;
   //ref.pos.x = cos(psi)*ref.pos_local.x - sin(psi)*ref.pos_local.y+waypoints_dr[dr_fp.gate_nr].x;
   //ref.pos.y = sin(psi)*ref.pos_local.x + cos(psi)*ref.pos_local.y+waypoints_dr[dr_fp.gate_nr].y;
}

void clear_reference()
{
    ref.pos.x = 0.0;
    ref.pos.y = 0.0;
}
