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

#ifndef BEBOP_DRAG_X
#define BEBOP_DRAG_X -0.5
#endif

#ifndef BEBOP_DRAG_Y
#define BEBOP_DRAG_Y -0.5
#endif


#ifndef BEBOP_DRAG_Z
#define BEBOP_DRAG_Z -0.5
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

/* ----------------------------------------------------  Differential flatness controller ----------------------------------------------*/

void generate_polynomial_trajectory(float **c_p,float * c_v, float * c_a, float * c_j, 
		                            struct Point_constraints initial_constraints,
								   	struct Point_constraints final_constraints,
									float t0, float tf);
void feed_forward_controller(struct FloatRates * ptr_omega_ff, float * thrust_ff,float *c_v_x,float *c_v_y,float *c_v_z,
		                                                                 float *c_a_x,float *c_a_y,float *c_a_z,
							 											 float *c_j_x,float *c_j_y,float *c_j_z,
							                                             float **c_p_psi,float *c_v_psi,float t);
float get_position_reference(float ** ptr_c_p,float time);
float get_velocity_reference(float * ptr_c_v,float time);
float get_acceleration_reference(float * ptr_c_a,float time);
float get_jerk_reference(float * ptr_c_j,float time);
void cross_product_3(float * o,float *a,float *b);
void vector_scale_3(float *o,float *a,float k);

float c_p_x[6][1],c_p_y[6][1],c_p_z[6][1],c_p_psi[6][1];
float c_v_x[5],c_v_y[5],c_v_z[5],c_v_psi[5];
float c_a_x[4],c_a_y[4],c_a_z[4],c_a_psi[4];
float c_j_x[3],c_j_y[3],c_j_z[3],c_j_psi[3];


struct timeval time_df_0;
struct timeval time_df;
struct FloatRates omega_ff;

bool differential_flatness_controller(struct Point_constraints xf, struct Point_constraints yf,
		                              struct Point_constraints zf, struct Point_constraints psif,
									  float t0, float tf)
{
    MAKE_MATRIX_PTR(ptr_c_p_x, c_p_x, 6); MAKE_MATRIX_PTR(ptr_c_p_y, c_p_y, 6); MAKE_MATRIX_PTR(ptr_c_p_z, c_p_z, 6); MAKE_MATRIX_PTR(ptr_c_p_psi, c_p_psi, 6);
	if(controllerInUse != DIFFERENTIAL_FLATNESS_CONTROLLER)
	{
		controllerInUse = DIFFERENTIAL_FLATNESS_CONTROLLER;
		/* ----------------------------    real flight ------------------------------------------------*/
		/*
		struct Point_constraints x0 = {stateGetPositionNed_f()->x,stateGetSpeedNed_f()->x,stateGetAccelNed_f()->x};
		struct Point_constraints y0 = {stateGetPositionNed_f()->y,stateGetSpeedNed_f()->y,stateGetAccelNed_f()->y};
		struct Point_constraints z0 = {stateGetPositionNed_f()->z,stateGetSpeedNed_f()->z,stateGetAccelNed_f()->z};
		struct Point_constraints psi0 = {stateGetNedToBodyEulers_f()->psi, stateGetBodyRates_f()->r,0.0};
		*/
		/* ----------------------------    real flight ------------------------------------------------*/
		struct Point_constraints x0 = {0.0,0.0,0.0};
		struct Point_constraints y0 = {0.0,0.0,0.0};
		struct Point_constraints z0 = {-1.5,0.0,0.0};
		struct Point_constraints psi0 = {0.0,0.0,0.0};
		generate_polynomial_trajectory(ptr_c_p_x,c_v_x,c_a_x,c_j_x,x0,xf,t0,tf);
		generate_polynomial_trajectory(ptr_c_p_y,c_v_y,c_a_y,c_j_y,y0,yf,t0,tf);
		generate_polynomial_trajectory(ptr_c_p_z,c_v_z,c_a_z,c_j_z,z0,zf,t0,tf);
		generate_polynomial_trajectory(ptr_c_p_psi,c_v_psi,c_a_psi,c_j_psi,psi0,psif,t0,tf);
		gettimeofday(&time_df_0, 0);
	}

	gettimeofday(&time_df, 0);
    float currentDeltaT = timedifference_msec(time_df_0,time_df)/1000.0;
	struct FloatRates rate_ff;
	float thrust_ff;


	feed_forward_controller(&rate_ff, &thrust_ff,c_v_x,c_v_y,c_v_z,c_a_x,c_a_y,c_a_z,
			c_j_x,c_j_y,c_j_z, ptr_c_p_psi,c_v_psi,currentDeltaT);

	/*
	if(currentDeltaT < 10.0)
		printf("Time = %f, p = %f, q = %f, r = %f, T = %f\n",currentDeltaT,rate_ff.p,rate_ff.q,rate_ff.r,thrust_ff);
		*/


	return true;
}

void feed_forward_controller(struct FloatRates * ptr_omega_ff, float * thrust_ff,float *c_v_x,float *c_v_y,float *c_v_z,
		                                                                 float *c_a_x,float *c_a_y,float *c_a_z,
							 											 float *c_j_x,float *c_j_y,float *c_j_z,
							                                             float **c_p_psi,float *c_v_psi,float t)
{
	float v[3] = {get_velocity_reference(c_v_x,t),get_velocity_reference(c_v_y,t),get_velocity_reference(c_v_z,t)};
	float a[3] = {get_acceleration_reference(c_a_x,t),get_acceleration_reference(c_a_y,t),get_acceleration_reference(c_a_z,t)};
	float jerk[3] = {get_jerk_reference(c_j_x,t),get_jerk_reference(c_j_y,t),get_jerk_reference(c_j_z,t)};
	float psi = get_position_reference(c_p_psi,t);
	float dPsi = get_velocity_reference(c_v_psi,t);
	float x_c[3] = {cos(psi), sin(psi), 0};
	float y_c[3] = {-sin(psi), cos(psi), 0};
	float z_w[3] = {0,0,1};

	/*
	printf("c_v_x = %f,%f,%f]\n",c_v_x[0],c_v_x[1],c_v_x[2]);
	printf("v = [%f,%f,%f]\n",v[0],v[1],v[2]);
	printf("jerk = [%f,%f,%f]\n",jerk[0],jerk[1],jerk[2]);
	*/

	float g[3], drag_x[3],drag_y[3],drag_z[3],g_plus_drag_x[3],g_plus_drag_y[3],g_plus_drag_z[3],alpha[3],beta[3],gamma[3];
	float x_b[3],y_b[3],z_b[3];
	vector_scale_3(g,z_w,9.8);
	vector_scale_3(drag_x,v,BEBOP_DRAG_X);
	float_vect_sum(g_plus_drag_x,g,drag_x,3);
	float_vect_diff(alpha,a,g_plus_drag_x,3);
	/*
	printf("z_w = [%f,%f,%f]\n\n",z_w[0],z_w[1],z_w[2]);
	printf("g = [%f,%f,%f]\n\n",g[0],g[1],g[2]);
	printf("drag_x = [%f,%f,%f]\n\n",drag_x[0],drag_x[1],drag_x[2]);
	printf("g + drag_x= [%f,%f,%f]\n\n",g_plus_drag_x[0],g_plus_drag_x[1],g_plus_drag_x[2]);
	printf("a = [%f,%f,%f]\n",a[0],a[1],a[2]);
	printf("alpha = [%f,%f,%f]\n\n\n\n",alpha[0],alpha[1],alpha[2]);
	*/

	vector_scale_3(drag_y,v,BEBOP_DRAG_Y);
	float_vect_sum(g_plus_drag_y,g,drag_y,3);
	float_vect_diff(beta,a,g_plus_drag_y,3);
	/*
	printf("beta = [%f,%f,%f]\n\n\n\n",beta[0],beta[1],beta[2]);
	*/

	float alpha_cross_yc[3], xb_cross_beta[3];
	cross_product_3(alpha_cross_yc,alpha,y_c);
	vector_scale_3(x_b,alpha_cross_yc,1.0/float_vect_norm(alpha_cross_yc,3));
	/*
	printf("x_b= [%f,%f,%f]\n\n\n\n",x_b[0],x_b[1],x_b[2]);
	*/

	cross_product_3(xb_cross_beta,x_b,beta);
	vector_scale_3(y_b,xb_cross_beta,1.0/float_vect_norm(xb_cross_beta,3));
	cross_product_3(z_b,x_b,y_b);

	/*
	printf("y_b= [%f,%f,%f]\n\n\n\n",y_b[0],y_b[1],y_b[2]);
	printf("z_b= [%f,%f,%f]\n\n\n\n",z_b[0],z_b[1],z_b[2]);
	*/
	
	vector_scale_3(drag_z,v,BEBOP_DRAG_Z);
	float_vect_sum(g_plus_drag_z,g,drag_z,3);
	float_vect_diff(gamma,a,g_plus_drag_z,3);
	*thrust_ff = float_vect_dot_product(z_b,gamma,3);
	//printf("thrust_ff =  %f\n\n\n\n",*thrust_ff);

	float yc_cross_zb[3];
	cross_product_3(yc_cross_zb,y_c,z_b);
	float A[3][3] = {{0,
               		  *thrust_ff+(BEBOP_DRAG_Z - BEBOP_DRAG_X)*(float_vect_dot_product(z_b,v,3)),
				 	  (BEBOP_DRAG_X-BEBOP_DRAG_Y)*(float_vect_dot_product(z_b,v,3))},
		             {*thrust_ff-(BEBOP_DRAG_Y - BEBOP_DRAG_Z)*(float_vect_dot_product(z_b,v,3)),
				       0,
					   -(BEBOP_DRAG_X - BEBOP_DRAG_Y)*(float_vect_dot_product(x_b,v,3))},
		             {0.0,
				 	 -float_vect_dot_product(y_c,z_b,3),
					 float_vect_norm(yc_cross_zb,3)}};
	float b[3][1] = {{float_vect_dot_product(x_b,jerk,3) - BEBOP_DRAG_X * float_vect_dot_product(x_b,a,3)},
		            {-float_vect_dot_product(y_b,jerk,3) + BEBOP_DRAG_Y * float_vect_dot_product(y_b,a,3)},
	                {dPsi * float_vect_dot_product(x_c,x_b,3)}};
	float U[3][3],W[3],V[3][3],angular_rate[3][1];
    MAKE_MATRIX_PTR(ptr_angular_rate, angular_rate, 3)
    MAKE_MATRIX_PTR(pA, A, 3);
    MAKE_MATRIX_PTR(pV, V, 3);
    MAKE_MATRIX_PTR(pb, b, 3);
    pprz_svd_float(pA,W,pV,3,3);
    pprz_svd_solve_float(ptr_angular_rate,pA,W,pV,pb,3,3,1);
	ptr_omega_ff->p = angular_rate[0][0];
	ptr_omega_ff->q = angular_rate[1][0];
	ptr_omega_ff->r = angular_rate[2][0];
	//printf("omegai = [%f,%f,%f]\n\n\n\n",ptr_omega_ff->p,ptr_omega_ff->q,ptr_omega_ff->r);
}

void cross_product_3(float * o,float *a,float *b)
{
	o[0] = a[1]*b[2]-a[2]*b[1];
	o[1] = a[2]*b[0]-a[0]*b[2];
	o[2] = a[0]*b[1]-a[1]*b[0];
}

void vector_scale_3(float *o,float *a,float k)
{
	for(int i=0;i<3;i++)
		*(o+i) = *(a+i)*k; 
}

float get_position_reference(float ** ptr_c_p,float time)
{
	float sum = 0;
	for(int i = 0; i<6; i++) sum += ptr_c_p[i][0]*pow(time,5-i); 
	return sum;
}



void generate_polynomial_trajectory(float **c_p,float * c_v, float * c_a, float * c_j, 
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
    pprz_svd_solve_float(c_p,pA,W,pV,pb,6,6,1);
	c_v[0] = 5*c_p[0][0]; c_v[1] = 4*c_p[1][0];c_v[2] = 3*c_p[2][0]; c_v[3] = 2*c_p[3][0]; c_v[4] = c_p[4][0];
    c_a[0] = 20 * c_p[0][0]; c_a[1] = 12 * c_p[1][0]; c_a[2] = 6 * c_p[2][0]; c_a[3] = 2*c_p[3][0];
    c_j[0] = 60 * c_p[0][0]; c_j[1] = 24 * c_p[1][0]; c_j[2] = 6* c_p[2][0];	

}


float get_velocity_reference(float * ptr_c_v,float time)
{
	float sum = 0;
	for(int i = 0; i<5; i++) sum += ptr_c_v[i]*pow(time,4-i); 
	return sum;
}

float get_acceleration_reference(float * ptr_c_a,float time)
{
	float sum = 0;
	for(int i = 0; i<4; i++) sum += ptr_c_a[i]*pow(time,3-i); 
	return sum;
}


float get_jerk_reference(float * ptr_c_j,float time)
{
	float sum = 0;
	for(int i = 0; i<3; i++) sum += ptr_c_j[i]*pow(time,2-i); 
	return sum;
}

/* ----------------------------------------------------  Differential flatness controller end ----------------------------------------------*/
