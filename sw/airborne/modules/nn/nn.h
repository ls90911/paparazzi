#ifndef _NN_H
#define _NN_H
#include "nn_params.h"

void compute_control(float **ptr_arr_1, float **ptr_arr_2);
extern void nn(float state[NUM_STATE_VARS], float control[NUM_CONTROL_VARS]);

#endif
