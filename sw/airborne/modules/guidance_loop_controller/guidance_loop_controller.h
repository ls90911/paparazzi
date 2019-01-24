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
 * @file "modules/guidance_loop_controller/guidance_loop_controller.h"
 * @author Shuo Li
 * This module includes all controllers that calculate low level command to attitude loop or even lower level loop.
 */




#ifndef GUIDANCE_LOOP_CONTROLLER_H
#define GUIDANCE_LOOP_CONTROLLER_H

#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/scheduleloop/scheduleloop.h"
#include "modules/guidance_loop_controller/guidance_loop_controller.h"
#include "modules/guidance_loop_controller/guidance_loop_controller.h"

#ifndef SCALING_COEDD
#define SCALING_COEFF 1.0  /* squared neighbourhood radius for using the PD */
#endif

#ifndef NN_PLUS_PD
#define NN_PLUS_PD FALSE 
#endif

enum ControllerInUse {NO_CONTROLLER,CONTROLLER_HOVER_WITH_OPTITRACK,CONTROLLER_NN_CONTROLLER,CONTROLLER_GO_TO_POINT} ;

struct NN_CMD {
    float thrust_ref;
    float rate_ref;
};

struct NN_STATE
{
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
	float theta;
	float phi;
	float psi;
	float p;
	float q;
	float r;
};

struct ADAPT_HOVER_COEFFCIENT
{
	int counter;
	int32_t sumDeltaT;
};

extern bool hover_with_optitrack(float hoverTime);
extern void nn_controller(float desired_x,float desired_z);
extern bool go_to_point(float desired_x,float desired_y,float desired_z,float desired_heading);

extern bool flagNN;
extern enum ControllerInUse controllerInUse;
extern struct NN_CMD nn_cmd;

extern struct FloatVect3 pos_OT;
extern struct FloatVect3 vel_OT;
extern struct FloatVect3 pos_NED;
extern struct FloatVect3 vel_NED;
extern struct FloatVect3 pos_NWU;
extern struct FloatVect3 vel_NWU;
extern float nn_time;

extern float psi_c;
extern float nn_x_sp;
extern float nn_z_sp;
#endif

