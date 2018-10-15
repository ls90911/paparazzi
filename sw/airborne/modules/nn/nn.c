#include <math.h>
#include "nn.h"
void preprocess_input(float input[]);
void postprocess_output(float control[]);
void layer_dense(float input[], float output[], int dense_layer_count);
void layer_relu(float activation[], int dense_layer_count);
void layer_tanh(float activation[], int dense_layer_count);
void nn_predict(float **arr_1, float **arr_2);

void preprocess_input(float input[]) {
    int i;
    for (i = 0; i < NUM_STATE_VARS; i++) {
        input[i] = (input[i] - INPUT_SCALER_PARAMS[0][i]) / INPUT_SCALER_PARAMS[1][i];
    }
}

void postprocess_output(float control[]) {
    int i;
    float out;
    for (i = 0; i < NUM_CONTROL_VARS; i++) {
        out = (control[i] - OUTPUT_SCALER_PARAMS[0][i]) / OUTPUT_SCALER_PARAMS[1][i];
        control[i] = out * OUTPUT_SCALER_PARAMS[2][i] + OUTPUT_SCALER_PARAMS[3][i];
    }
}

void layer_dense(float input[], float output[], int dense_layer_count) {
    int i, j;
    for (i = 0; i < LAYER_DIMS[dense_layer_count+1]; i++) {
        output[i] = BIASES[dense_layer_count][i];
        for (j = 0; j < LAYER_DIMS[dense_layer_count]; j++) {
            output[i] += WEIGHTS[dense_layer_count][i][j] * input[j];
        }
    }
}

void layer_relu(float activation[], int dense_layer_count) {
    int i;
    for (i = 0; i < LAYER_DIMS[dense_layer_count]; i++) {
        activation[i] = (activation[i] < 0) ? 0 : activation[i];
    }
}

void layer_tanh(float activation[], int dense_layer_count) {
    int i;
    for (i = 0; i < LAYER_DIMS[dense_layer_count]; i++) {
        activation[i] = tanh(activation[i]);
    }
}

void nn_predict(float **arr_1, float **arr_2) {
    float *prev_layer_output = *arr_1;
    float *curr_layer_output = *arr_2;
    float *tmp;
    int i, dense_layer_count = 0;

    for (i = 0; i < NUM_ALL_LAYERS; i++) {
        Layer layer = LAYER_TYPES[i];
        switch (layer) {
            case DENSE:
                layer_dense(prev_layer_output, curr_layer_output, dense_layer_count);
                dense_layer_count++;
                tmp = prev_layer_output;
                prev_layer_output = curr_layer_output;
                curr_layer_output = tmp;
                break;
            case RELU:
                layer_relu(prev_layer_output, dense_layer_count);
                break;
            case TANH:
                layer_tanh(prev_layer_output, dense_layer_count);
                break;
        }
    }

    /* nn output goes back into arr_1 */
    *arr_1 = prev_layer_output;
    *arr_2 = curr_layer_output;
}

void compute_control(float **ptr_arr_1, float **ptr_arr_2) {
    preprocess_input(*ptr_arr_1);
    nn_predict(ptr_arr_1, ptr_arr_2);
    postprocess_output(*ptr_arr_1);
}

/* This function is for testing purposes only and should not be used for */
/* embedded implementation since it (unnecessarily) allocates */
/* two arrays every time */
void nn(float state[NUM_STATE_VARS], float control[NUM_CONTROL_VARS]) {
	/* allocate memory for two arrays required by compute_control() */
	/* number of array elements must be at least no. of units in widest layer */
	float arr_1_tmp[MAX_LAYER_DIMS];
	float arr_2_tmp[MAX_LAYER_DIMS];
	float *arr_1 = (float *) arr_1_tmp; /* replace with malloc if necessary */
	float *arr_2 = (float *) arr_2_tmp;
	/* note the block of memory pointed to by the above may swap as a result */
	/* of calling compute_control() */

	/* on function call arr_1 contains state values */
	/* on completion, arr_1 contains control values */
	memcpy(arr_1, state, NUM_STATE_VARS * sizeof(float));
	compute_control(&arr_1, &arr_2);
	memcpy(control, arr_1, NUM_CONTROL_VARS * sizeof(float));
	/* TODO: replace memcpy with straightforward for-loop assignment? */
}

