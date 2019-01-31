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

extern struct dronerace_control_struct dr_control;
extern struct pid_term_struct pid_term;

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
#endif
