
#include "flightplan.h"
#include "filter.h"
#include "ransac.h"
#include "std.h"
#include "stdio.h"
#include "state.h"
#include "control.h"

struct dronerace_fp_struct dr_fp;
void generate_waypoints_from_gates(void);

int flagHighOrLowGate;
float dist_2_gate;
int num_lap;

const struct dronerace_flightplan_item_struct gates[MAX_GATES] = {
  //  X-coordinate  Y-coordinate  Z-coordinate   Psi-gate          Speed    Type-of-gate  Brake-at-gate   Distance-after gate       both side
  {   4.0,          0.0,          -1.5,          RadOfDeg(0),      1.2f,    REGULAR,      NO_BRAKE,       1.0,                      0},
  {   5.0,          5.0,          -1.5,          RadOfDeg(90),      1.2f,    REGULAR,      NO_BRAKE,       1.0,                      0},
  {   1.0,          6.0,          -1.5,          RadOfDeg(180),      1.2f,    REGULAR,      NO_BRAKE,       1.0,                      0},
  {   0.0,          1.0,          -1.5,          RadOfDeg(270),      1.2f,    REGULAR,      NO_BRAKE,       2.0,                      0},
};

struct dronerace_flightplan_item_struct waypoints_dr[MAX_GATES];

static void update_gate_setpoints(void)
{
  dr_fp.gate_x     = gates[dr_fp.gate_nr].x;
  dr_fp.gate_y     = gates[dr_fp.gate_nr].y;
  dr_fp.gate_z     = gates[dr_fp.gate_nr].z;
  dr_fp.gate_psi   = gates[dr_fp.gate_nr].psi;
  dr_fp.gate_speed = gates[dr_fp.gate_nr].speed;
}

void flightplan_reset()
{
  // Current Gate
  dr_fp.gate_nr = 0;
  update_gate_setpoints();

  // Navigation Setpoint
  dr_fp.x_set = 3;
  dr_fp.y_set = 0;
  dr_fp.z_set = 0;
  dr_fp.psi_set = 0;
  num_lap = 0;

  generate_waypoints_from_gates();
}


#define DISTANCE_GATE_NOT_IN_SIGHT  2.5f
#define DISTANCE_ARRIVED_AT_WP    1.0f

void flightplan_run(void)
{
  float dist = 0.0;
  float correctedX, correctedY;
  float dx, dy;

  // Get current gate position
  update_gate_setpoints();

  dr_fp.x_set = waypoints_dr[dr_fp.gate_nr].x;
  dr_fp.y_set = waypoints_dr[dr_fp.gate_nr].y;
  dr_fp.z_set = dr_fp.gate_z;

  // Estimate distance to the gate
  correctedX = dr_state.x + dr_ransac.corr_x;
  correctedY = dr_state.y + dr_ransac.corr_y;

  dx = waypoints_dr[dr_fp.gate_nr].x - correctedX;
  dy = waypoints_dr[dr_fp.gate_nr].y - correctedY;
  dist = sqrt((dx * dx) + (dy * dy));

  // Align with current gate
  dr_fp.psi_set = dr_fp.gate_psi + num_lap*2*3.14;

  dist_2_gate = (dr_fp.x_set- filteredX) * (dr_fp.x_set- filteredX) + (dr_fp.y_set- filteredY) *
                (dr_fp.y_set- filteredY);

  // If too close to the gate to see the gate, heading to next gate
  if (dist_2_gate < DISTANCE_GATE_NOT_IN_SIGHT) {
    if ((dr_fp.gate_nr + 1) < MAX_GATES) {
      dr_fp.psi_set = (gates[dr_fp.gate_nr+1].psi)+num_lap*2*3.14;
    }
	else
	{
      dr_fp.psi_set = (gates[0].psi)+(num_lap+1)*2*3.14;
	}
  }


  // If close to desired position, switch to next
  if (dist < DISTANCE_ARRIVED_AT_WP) {
    dr_fp.gate_nr ++;
    if (dr_fp.gate_nr >= MAX_GATES) {
      dr_fp.gate_nr = 0;
	  num_lap++;
    }

  }
}

// #define DEBUG_WP_GENERATION
void generate_waypoints_from_gates()
{
  int i;
#ifdef DEBUG_WP_GENERATION
  if (debug) {

    char filename[128];
    FILE *fp;
    sprintf(filename, "%06d.txt", 1111);
    fp = fopen(filename, "w");
    fprintf(fp, "gate_x,gate_y,gate_z,gate_psi,gate_tpye,gate_brake,gate_after_distance\n");
    for (int i = 0; i < MAX_GATES; i++) {
      fprintf(fp, "%f,%f,%f,%f,%d,%d,%f\n", gates[i].x,
              gates[i].y,
              gates[i].z,
              gates[i].psi,
              gates[i].type,
              gates[i].brake,
              gates[i].distance_after_gate
             );
    }

    fprintf(fp, "\n\n\n");
    fclose(fp);
  }
#endif

  for (i = 0; i < MAX_GATES; i++) {
    float d = gates[i].distance_after_gate;
    if (gates[i].type == VIRTUAL) {
      waypoints_dr[i].x = gates[i].x;
      waypoints_dr[i].y = gates[i].y;
    } else {
      waypoints_dr[i].x = cos(gates[i].psi) * d + gates[i].x;
      waypoints_dr[i].y = sin(gates[i].psi) * d + gates[i].y;
    }
    waypoints_dr[i].z = gates[i].z;
    waypoints_dr[i].psi = gates[i].psi;
    waypoints_dr[i].speed = gates[i].speed;
    waypoints_dr[i].type = gates[i].type;
    waypoints_dr[i].brake = gates[i].brake;
    waypoints_dr[i].distance_after_gate = gates[i].distance_after_gate;
    waypoints_dr[i].both_side = gates[i].both_side;
  }

#ifdef DEBUG_WP_GENERATION
  fp = fopen(filename, "a");
  fprintf(fp, "wp_x,wp_y,wp_z,wp_psi,wp_tpye,wp_brake,wp_after_distance\n");
  for (int i = 0; i < MAX_GATES; i++) {
    fprintf(fp, "%f,%f,%f,%f,%d,%d,%f\n", waypoints_dr[i].x,
            waypoints_dr[i].y,
            waypoints_dr[i].z,
            waypoints_dr[i].psi,
            waypoints_dr[i].type,
            waypoints_dr[i].brake,
            waypoints_dr[i].distance_after_gate
           );
  }

  fclose(fp);
#endif

}
