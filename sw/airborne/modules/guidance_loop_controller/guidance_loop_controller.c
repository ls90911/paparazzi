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
 * @file "modules/guidance_loop_controller/guidance_loop_controller.c"
 * @author Shuo Li
 * This module includes all controllers that calculate low level command to attitude loop or even lower level loop.
 */

#include "modules/guidance_loop_controller/guidance_loop_controller.h"
#include "modules/guidance_h_module/guidance_h_module.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/nn/nn.h"
#include "modules/nn/nn_params.h"
#include "stdio.h"
#include "state.h"
#include<sys/time.h>

#include "math/pprz_matrix_decomp_float.h"
//#include "firmwares/rotorcraft/guidance/guidance_indi.h"

#ifndef GRAVITY_FACTOR
#define GRAVITY_FACTOR 9.81
#endif

#ifndef BEBOP_MASS 
#define BEBOP_MASS 0.389 
#endif

struct FloatVect3 sp_accel = {0.0, 0.0, 0.0};
float nominal_throttle = GUIDANCE_V_NOMINAL_HOVER_THROTTLE;
struct Debug_indi debug_indi;  // not used. For log
struct Debug_PID_Acceleration debug_pid_acc;

/*---------------------------From Kirk --------------------------------*/
#include "filters/low_pass_filter.h"

float thrust_effectiveness = 0.05f; // transfer function from G to thrust percentage
float error_integrator = 0.f;
//#include "modules/nn/nn_params.h"
#ifndef NN_FILTER_CUTOFF
#define NN_FILTER_CUTOFF 1.5f
#endif

#ifndef THRUST_P_GAIN
#define THRUST_P_GAIN -1.7
#endif


#ifndef THRUST_D_GAIN
#define THRUST_D_GAIN -0.3
#endif

#ifndef THRUST_I_GAIN
#define THRUST_I_GAIN -0.1
#endif

float thrust_p_gain = THRUST_P_GAIN;
float thrust_d_gain = THRUST_D_GAIN;
float thrust_i_gain = THRUST_I_GAIN;

Butterworth2LowPass accel_ned_filt;

/*--------------------------------------------------------------------*/

enum ControllerInUse controllerInUse;
struct Pos hoverPos;
struct Euler hoverEuler;
bool flagNN;
struct NN_CMD nn_cmd;
struct NN_STATE nn_state;
struct ADAPT_HOVER_COEFFCIENT hover_coefficient;
float scale_factor;

struct FloatRMat R_OPTITRACK_2_NED, R_NED_2_NWU;
struct FloatEulers eulersOT2NED = {0.0,0.0,-0.0/180.0*3.14};
struct FloatEulers eulersNED2BWU = {3.14,0.0,0.0};
struct FloatVect3 pos_OT,vel_OT,pos_NED,vel_NED,pos_NWU,vel_NWU;
float nn_time;
float temp_time;
float psi_c;
struct timeval t0;
struct timeval t1;
struct timeval NN_start;
float nn_x_sp,nn_z_sp;


bool hover_with_optitrack(float hoverTime)
{
    if(controllerInUse!= CONTROLLER_HOVER_WITH_OPTITRACK)
    {
            controllerInUse= CONTROLLER_HOVER_WITH_OPTITRACK;
            clearClock(2);
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            hoverPos.x = stateGetPositionNed_f()->x;
            hoverPos.y= stateGetPositionNed_f()->y;
            hoverPos.z = stateGetPositionNed_f()->z;
            hoverEuler.psi= 0;
            printf("hover is initialized\n");
            printf("time 2 is %f\n",getTime(2));
            flagNN = false;
			float sample_time = 1.f / 100.0;
			float tau = 1.f / (2.f * M_PI * NN_FILTER_CUTOFF);
			init_butterworth_2_low_pass(&accel_ned_filt, tau, sample_time, 0.0);
    }

    // -------------for log -----------------------

    float_rmat_of_eulers_321(&R_OPTITRACK_2_NED,&eulersOT2NED);
    float_rmat_of_eulers_321(&R_NED_2_NWU,&eulersNED2BWU);
    pos_OT.x = stateGetPositionNed_f()->x;
    pos_OT.y = stateGetPositionNed_f()->y;
    pos_OT.z = stateGetPositionNed_f()->z;
    vel_OT.x = stateGetSpeedNed_f()->x;
    vel_OT.y = stateGetSpeedNed_f()->y;
    vel_OT.z = stateGetSpeedNed_f()->z;
    
   float_rmat_transp_vmult(&pos_NED, &R_OPTITRACK_2_NED, &pos_OT);
   float_rmat_transp_vmult(&vel_NED, &R_OPTITRACK_2_NED, &vel_OT);

   float_rmat_transp_vmult(&pos_NWU, &R_NED_2_NWU, &pos_NED);
   float_rmat_transp_vmult(&vel_NWU, &R_NED_2_NWU, &vel_NED);
   // ----------------------------------------------
   struct NedCoor_f *accel = stateGetAccelNed_f();
   update_butterworth_2_low_pass(&accel_ned_filt, accel->z);
   float filtered_az = (accel_ned_filt.o[0]-GRAVITY_FACTOR)/cosf(stateGetNedToBodyEulers_f()->theta);
   debug_pid_acc.az_filtered = filtered_az;
   
   guidance_h_set_guided_pos(0.0, 0.0); 
   guidance_v_set_guided_z(-1.5);
   guidance_h_set_guided_heading(0.0);

    if(getTime(2)>hoverTime)
    {
        return true;
    }
    else
    {
        return false;
    }

}

float timedifference_msec(struct timeval t0, struct timeval t1)
{
	return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
}

void nn_controller(float desired_x,float desired_z)
{
    if(controllerInUse!= CONTROLLER_NN_CONTROLLER)
    {
            controllerInUse = CONTROLLER_NN_CONTROLLER;
            clearClock(2);
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            flagNN = true;
            printf("[nn controle] nn controller is activated]");

	    float_rmat_of_eulers_321(&R_OPTITRACK_2_NED,&eulersOT2NED);
	    float_rmat_of_eulers_321(&R_NED_2_NWU,&eulersNED2BWU);

	    hoverPos.x = stateGetPositionNed_f()->x;
	    hoverPos.y = stateGetPositionNed_f()->y;
	    hoverPos.z = stateGetPositionNed_f()->z;

	    guidance_h_set_guided_heading(0);
	    guidance_v_set_guided_z(desired_z);
	    gettimeofday(&NN_start, 0);
		float tau = 1.f / (2.f * M_PI * NN_FILTER_CUTOFF);
		error_integrator = 0.0;
    }

    nn_x_sp = desired_x;
    nn_z_sp = desired_z;

   guidance_h_set_guided_pos(desired_x,0.0); 
   guidance_h_set_guided_heading(0.0);
    //set_z_ref(desired_z);
    int time_int;
    gettimeofday(&t1, 0);
    temp_time = timedifference_msec(NN_start,t1);
    time_int = temp_time/1000;
    int temp = time_int/10%10;
	    //guidance_v_set_guided_z(-0.5-0.5*(temp));

    // transform coordinate from Optitrack frame to NED frame of cyberzoo and then to North-west-up frame

    pos_OT.x = stateGetPositionNed_f()->x;
    pos_OT.y = stateGetPositionNed_f()->y;
    pos_OT.z = stateGetPositionNed_f()->z;
    vel_OT.x = stateGetSpeedNed_f()->x;
    vel_OT.y = stateGetSpeedNed_f()->y;
    vel_OT.z = stateGetSpeedNed_f()->z;
    
    float_rmat_transp_vmult(&pos_NED, &R_OPTITRACK_2_NED, &pos_OT);
    float_rmat_transp_vmult(&vel_NED, &R_OPTITRACK_2_NED, &vel_OT);

    float_rmat_transp_vmult(&pos_NWU, &R_NED_2_NWU, &pos_NED);
    float_rmat_transp_vmult(&vel_NWU, &R_NED_2_NWU, &vel_NED);

   // prepare current states to feed NN
    float state[NUM_STATE_VARS] = {desired_x-pos_NWU.x, -vel_NWU.x, pos_NWU.z+desired_z, vel_NWU.z, -stateGetNedToBodyEulers_f()->theta,-stateGetBodyRates_f()->q};
    float control[NUM_CONTROL_VARS];
    gettimeofday(&t0, 0);
    //nn_stable(state, control);
    nn(state, control);
//   nested_control(state, control) ;
    gettimeofday(&t1, 0);
    nn_time = timedifference_msec(t0,t1);

    //nn_cmd.thrust_ref = control[0] ;
    //nn_cmd.rate_ref = -control[1] ;
	float F_min = 1.76;
	float F_max = 2.35;
	
	nn_cmd.FL = F_min+(F_max-F_min)*control[0];
	nn_cmd.FR = F_min+(F_max-F_min)*control[1];

	static float error_integrator = 0.f;
	static float previous_error = 0.f;
	struct NedCoor_f *accel = stateGetAccelNed_f();
	update_butterworth_2_low_pass(&accel_ned_filt, accel->z);
	sp_accel.z = -(nn_cmd.FL+nn_cmd.FR)/BEBOP_MASS;
	
	/*
	if(getTime(2)<2)
	{
		sp_accel.z = -10.5;
	}
	else if(getTime(2)<3)
	{
		sp_accel.z = -6.0;
	}
	else
	{
		sp_accel.z = -12.0;
	}
	*/

	float filtered_az = (accel_ned_filt.o[0]-GRAVITY_FACTOR)/cosf(stateGetNedToBodyEulers_f()->theta);
	float error_az = sp_accel.z - filtered_az;
	error_integrator += error_az / 100.0;
	float thrust_sp = (error_az*thrust_p_gain + error_integrator*thrust_i_gain + thrust_d_gain*(error_az-previous_error)/100.0)*thrust_effectiveness + nominal_throttle;
	//printf("[guidance_loop_controller] nominal_throttle = %f\n",nominal_throttle);
	guidance_v_set_guided_th(thrust_sp);

	previous_error = error_az;
	
	/* --------------------- for log ---------------------------*/

	debug_pid_acc.az_sp = sp_accel.z;
	debug_pid_acc.az_error = error_az;
	debug_pid_acc.az_filtered = filtered_az;
	debug_pid_acc.thrust_sp = thrust_sp;
	debug_pid_acc.thrust_nominal = nominal_throttle;
	/* --------------------- for log ---------------------------*/
}

bool go_to_point(float desired_x,float desired_y,float desired_z,float desired_heading)
{
    if(controllerInUse != CONTROLLER_GO_TO_POINT)
    {
            controllerInUse = CONTROLLER_GO_TO_POINT ;
            clearClock(2);
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            flagNN = false;
    }
    guidance_h_set_guided_pos(desired_x, desired_y);
    guidance_v_set_guided_z(desired_z);
    guidance_h_set_guided_heading(desired_heading);

    float error_x =fabs(stateGetPositionNed_f()->x - desired_x);
    float error_y =fabs(stateGetPositionNed_f()->y - desired_y);
    float error_z =fabs(stateGetPositionNed_f()->z - desired_z);
    //set_z_ref(desired_z);
    if(error_x+error_y+error_z < 0.2)
    {
        return true;
    }
    else
    {
        return false;
    }

}


void generate_polynomial_trajectory(float *c_p,float * c_v, float * c_a, float * c_j, 
		                            struct Point_constraints initial_constraints,
								   	struct Point_constraints final_constraints,
									float t0, float tf)
{
	float A[6][6] = {{pow(t0,5),		pow(t0,4),		pow(t0,3),		pow(t0,2),		pow(t0,1),		1},
	                 {pow(tf,5),		pow(tf,4),		pow(tf,3),		pow(tf,2),		pow(tf,1),		1},
					 {5*pow(t0,4),		4*pow(t0,3),	3*pow(t0,2),	2*pow(t0,1),	1,				0},
					 {5*pow(tf,4),		4*pow(tf,3),	3*pow(tf,2),	2*pow(tf,1),	1,				0},
					 {20*pow(t0,3),		12*pow(t0,2),	6*pow(t0,1),	2,				0,				0},
					 {20*pow(tf,3),		12*pow(tf,2),	6*pow(tf,1),	2,				0,				0}};
	float b[6][1] = {{initial_constraints.p},{final_constraints.p},{initial_constraints.v},{final_constraints.v},
		             {initial_constraints.a},{final_constraints.a}};

	float U[6][6],W[6],V[6][6];
    MAKE_MATRIX_PTR(pA, A, 6);
    MAKE_MATRIX_PTR(pV, V, 6);
    MAKE_MATRIX_PTR(pb, b, 6);
    pprz_svd_float(pA,W,pV,6,6);
	float c[6][1];
    MAKE_MATRIX_PTR(pc, c, 6);
    pprz_svd_solve_float(pc,pA,W,pV,pb,6,6,1);
	for(int i = 0; i < 6;i++) c_p[i] = c[i][0];

}

bool differential_flatness_controller(struct Point_constraints xf, struct Point_constraints yf,
		                              struct Point_constraints zf, struct Point_constraints psif,
									  float t0, float tf)
{
	if(controllerInUse != DIFFERENTIAL_FLATNESS_CONTROLLER)
	{
		controllerInUse = DIFFERENTIAL_FLATNESS_CONTROLLER;
		struct Point_constraints x0 = {stateGetPositionNed_f()->x,stateGetSpeedNed_f()->x,stateGetAccelNed_f()->x};
		struct Point_constraints y0 = {stateGetPositionNed_f()->y,stateGetSpeedNed_f()->y,stateGetAccelNed_f()->y};
		struct Point_constraints z0 = {stateGetPositionNed_f()->z,stateGetSpeedNed_f()->z,stateGetAccelNed_f()->z};
		struct Point_constraints psi0 = {stateGetNedToBodyEulers_f()->psi, stateGetBodyRates_f()->r,0.0};
	}
	struct Point_constraints x0 = {0.0,0.0,0.0};
	struct Point_constraints x_f_temp = {5.0,0.0,0};
	float c_p[6],c_v[5],c_a[4],c_j[3];
	generate_polynomial_trajectory(c_p,c_v,c_a,c_j,x0,x_f_temp,0,10);
	printf("c_p = [%f,%f,%f,%f,%f,%f\n",c_p[0],c_p[1],c_p[2],c_p[3],c_p[4],c_p[5]);
}

