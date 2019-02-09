#ifndef CONTROL_H_H_
#define CONTROL_H_H_
#ifndef  P_FORWARD
#define P_FORWARD 1.1
#endif

#ifndef  D_FORWARD
#define D_FORWARD 0.0
#endif

#ifndef  P_LATERAL
#define P_LATERAL 1.1
#endif

#ifndef  D_LATERAL
#define D_LATERAL 0.0
#endif

#ifndef K_P_X_LOCAL
#define K_P_X_LOCAL 1.0
#endif

#ifndef K_V_X_LOCAL
#define K_V_X_LOCAL 2.0 
#endif


#ifndef K_P_Y_LOCAL
#define K_P_Y_LOCAL 20.0 
#endif

#ifndef K_V_Y_LOCAL
#define K_V_Y_LOCAL 10.0 
#endif

#include "math/pprz_algebra_float.h"

struct dronerace_control_struct
{
  // States
  float psi_ref;    ///< maintain a rate limited speed set-point to smooth heading changes

  // Outputs to inner loop
  float phi_cmd;
  float theta_cmd;
  float psi_cmd;
  float z_cmd;
};

struct pid_term_struct
{
    float p_term_x;
    float p_term_y;
    float d_term_x;
    float d_term_y;
    float vx_cmd;
    float vy_cmd;
};

struct reference_generator_struct
{
    struct FloatVect2 pos;
    struct FloatVect2 vel;
    struct FloatVect2 pos_local;
    struct FloatVect2 vel_local;
    float k_p_x_local;
    float k_p_y_local;
    float k_v_x_local;
    float k_v_y_local;
};

struct indi_controller_struct
{
    float vx_cmd;
    float vy_cmd;
    float ax_cmd;
    float ay_cmd;
    float r_ref;
    float r_k_p;
    float psi_err;
    float r_bound;
	float x_err;
	float y_err;
	float previous_x_err;
	float previous_y_err;
	float z_sp;
	float vz_sp;
	float previous_vx_err;
	float previous_vy_err;
};

extern struct dronerace_control_struct dr_control;
extern struct pid_term_struct pid_term;
extern struct reference_generator_struct ref;
extern struct indi_controller_struct indi_ctrl;

extern void control_reset(void);
extern void control_run(void);

extern float k_p_vel_x;
extern float k_d_vel_x;
extern float k_p_vel_y;
extern float k_d_vel_y;

extern float k_p_pos_x;
extern float k_d_pos_x;
extern float k_p_pos_y;
extern float k_d_pos_y;
extern void reset_reference(void);
extern void reset_local_reference(void);
extern void reference_init(void);
extern void update_reference_run(void);
extern void clear_reference(void);
#endif
