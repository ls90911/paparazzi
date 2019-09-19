

#include "filter.h"
#include "fifo.h"
#include "ransac.h"
#include "flightplan.h"
#include <math.h>
#include "std.h"
#include "stdio.h"
#include "ransac.h"
#include <time.h>
#include "dronerace.h"
// to know if we are simulating:
#include "generated/airframe.h"
#include "modules/sensors/cameras/jevois_mavlink.h"


struct dronerace_state_struct dr_state;
struct dronerace_vision_struct dr_vision;
int detection_time_stamp;
void calibrate_detection(float *mx,float *my);
int assigned_gate = 0;


void filter_reset()
{
  // Time
  dr_state.time = 0.0f;

  // Position
  dr_state.x = 0.0f;
  dr_state.y = 0.0f;

  // Speed
  dr_state.vx = 0.0f;
  dr_state.vy = 0.0f;

  // Heading
  dr_state.psi = 0.0f;

  // Vision latency
  fifo_reset();
  ransac_reset();
}

float filteredX, filteredY, filteredVx,filteredVy;


// PREDICTION MODEL

#define DR_FILTER_GRAVITY  9.81
#if SIMULATE
#define DR_FILTER_DRAG  0.95
#define DR_FILTER_THRUSTCORR  0.8
#else
#define DR_FILTER_DRAG  0.5
#define DR_FILTER_THRUSTCORR  0.8
#endif

void filter_predict(float phi, float theta, float psi, float dt)
{
  BoundAbs(phi, RadOfDeg(50));
  BoundAbs(theta, RadOfDeg(50));
  float az = DR_FILTER_GRAVITY / cosf(theta * DR_FILTER_THRUSTCORR) / cosf(phi * DR_FILTER_THRUSTCORR);
  float abx =  sinf(-theta) * az;
  float aby =  sinf(phi)   * az;

  // Earth accelerations
  float ax =  cosf(psi) * abx - sinf(psi) * aby - dr_state.vx * DR_FILTER_DRAG ;
  float ay =  sinf(psi) * abx + cosf(psi) * aby - dr_state.vy * DR_FILTER_DRAG;


  get_time_stamp();
  // Velocity and Position
  dr_state.vx += ax * dt;
  dr_state.vy += ay * dt;
  dr_state.x += dr_state.vx * dt;
  dr_state.y += dr_state.vy * dt;

  // Time
  dr_state.time += dt;

  // Store psi for local corrections
  dr_state.psi = psi; // TODO: use psi command?

  // Store old states for latency compensation
  fifo_push(dr_state.x, dr_state.y, 0);

  // Check if Ransac buffer is empty
  ransac_propagate();

  float deltaT = dr_state.time - ransac_buf[get_index(0)].time;
  filteredX = dr_state.x + dr_ransac.corr_x + dr_ransac.corr_vx*deltaT;
  filteredY = dr_state.y + dr_ransac.corr_y + dr_ransac.corr_vy*deltaT;
  filteredVx = dr_state.vx + dr_ransac.corr_vx;
  filteredVy = dr_state.vy + dr_ransac.corr_vy;
}

float log_mx, log_my;
float mx, my;
int transfer_measurement_local_2_global(float *mx, float *my, float dx, float dy);
int filter_cnt = 55;

void filter_correct(void)
{
	filter_cnt++;
	//heart_beat = filter_cnt;
  // Retrieve oldest element of state buffer (that corresponds to current vision measurement) // TODO: should we not empirically determine the delay (is it now just guessed?)
  float sx, sy, sz;

  fifo_pop(&sx, &sy, &sz);

  // TODO: we should actually check that the determined height is not so different from the gate height, given that we are not looking at the jungle gate
  // With the check on dr_vision.dz, we want to exclude the detection of the gate botom part.
  //  && dr_vision.dz > -2.5
  if (gates[dr_fp.gate_nr].type != VIRTUAL) {



    //calibrate_detection(&mx,&my);
    assigned_gate = transfer_measurement_local_2_global(&mx, &my, dr_vision.dx, dr_vision.dy);

    //if (assigned_gate == dr_fp.gate_nr) {
    if (1) {



      // Push to RANSAC
      detection_time_stamp = get_time_stamp();
      ransac_push(dr_state.time, dr_state.x, dr_state.y, mx, my,detection_time_stamp);

      filteredX = dr_state.x + dr_ransac.corr_x;
      filteredY = dr_state.y + dr_ransac.corr_y;
      filteredVx = dr_state.vx + dr_ransac.corr_vx;
      filteredVy = dr_state.vy + dr_ransac.corr_vy;
      
      return;
    }
  }
}


int transfer_measurement_local_2_global(float *_mx, float *_my, float dx, float dy)
{
	int i;
	float min_distance = 9999;

	dr_state.assigned_gate_index = -1;

	for (i = 0; i < MAX_GATES; i++) {
		if (gates[i].type != VIRTUAL) {

			/*
			float exp_dx = gates[i].x - dr_state.x;
			float exp_dy = gates[i].y - dr_state.y;
			//float exp_yaw = scale_heading(gates[i].psi) - scale_heading(dr_state.psi);
			float exp_yaw = 0;
			float exp_dist = sqrtf(exp_dx * exp_dx + exp_dy * exp_dy);
			if (exp_dist == 0.0) {
				exp_dist = 0.0001f;
			}
			float exp_size =  1.4f * 340.0f / exp_dist;
			// dist = 1.4f * 340.0f / ((float)size);
			float exp_bearing = atan2(exp_dy, exp_dx);
			float exp_view = exp_bearing - dr_state.psi;
			if ((exp_view > -320.0f / 340.0f) && (exp_view < 320.0f / 340.0f)
					&& ((exp_yaw > -RadOfDeg(60.0f)) && (exp_yaw < RadOfDeg(60.0f)))
			   ) {

			   */
				float rot_dx = cosf(dr_state.psi) * dx -sinf(dr_state.psi) * dy;
				float rot_dy = sinf(dr_state.psi) * dx + cosf(dr_state.psi) * dy;

				float x = gates[i].x + rot_dx;
				float y = gates[i].y + rot_dy;
				float distance_measured_2_drone = 0;
				distance_measured_2_drone = (x - (dr_state.x + dr_ransac.corr_x)) * (x - (dr_state.x + dr_ransac.corr_x)) +
					(y - (dr_state.y + dr_ransac.corr_y)) * (y - (dr_state.y + dr_ransac.corr_y));
				if (distance_measured_2_drone < min_distance) {
					dr_state.assigned_gate_index = i;
					min_distance = distance_measured_2_drone;
					*_mx = x;
					*_my = y;
				}
			//}
		}
	}
	if(dr_state.assigned_gate_index == -1) {
		dr_state.assigned_gate_index = dr_fp.gate_nr;
	}

	return dr_state.assigned_gate_index;
}


int get_time_stamp()
{
/*
  struct timeval te;
  gettimeofday(&te, NULL); // get current time
  long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
  int timeStamp = milliseconds%100000;
  //printf("Timestamp: %d\n",timeStamp);
  return timeStamp;
  */
	return 1;
}


void calibrate_detection(float *mmx,float *mmy)
{
    float k0_x = -0.0;
    float k1_x = 0.0;
    float k2_x = -0.0;

    float k0_y = 0.0;
    float k1_y = 0.0;
    float k2_y = -0.0;

    float x = *mmx;
    float y = *mmy;

    *mmx = x + k0_x + k1_x*x+k2_x*x*x;
    *mmy = y + k0_y + k1_y*y+k2_y*y*y;

}


