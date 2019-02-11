

#include "filter.h"
#include "fifo.h"
#include "ransac.h"
#include "flightplan.h"
#include <math.h>
#include "std.h"
#include "stdio.h"
#include "ransac.h"
#include <time.h>
// to know if we are simulating:
#include "generated/airframe.h"


struct dronerace_state_struct dr_state;
struct dronerace_vision_struct dr_vision;
struct calibrate_ahrs_struct cali_ahrs = {0,0,0,0};
int detection_time_stamp;
void calibrate_detection(float *mx,float *my);
void calibrate_ahrs(void);
void calibrate_ahrs_init(void);

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
  calibrate_ahrs_init();
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
	if (cali_ahrs.is_ahrs_calibrated == 0)
	{
		calibrate_ahrs();
	}
  ////////////////////////////////////////////////////////////////////////
  // Body accelerations
  
	phi -= cosf(psi) *cali_ahrs.bias_north - sinf(psi)*cali_ahrs.bias_east;
	theta -= sinf(psi) *cali_ahrs.bias_north + cosf(psi)*cali_ahrs.bias_east;
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

void pushJungleGateDetection(void);

void filter_correct(void)
{
  // Retrieve oldest element of state buffer (that corresponds to current vision measurement) // TODO: should we not empirically determine the delay (is it now just guessed?)
  float sx, sy, sz;

  fifo_pop(&sx, &sy, &sz);

  // TODO: we should actually check that the determined height is not so different from the gate height, given that we are not looking at the jungle gate
  // With the check on dr_vision.dz, we want to exclude the detection of the gate botom part.
  //  && dr_vision.dz > -2.5
  if (gates[dr_fp.gate_nr].type != VIRTUAL) {



    //calibrate_detection(&mx,&my);
    int assigned_gate = transfer_measurement_local_2_global(&mx, &my, dr_vision.dx, dr_vision.dy);

    //printf("assigned gate = %d, gate nr = %d.\n", assigned_gate, dr_fp.gate_nr);

    if (assigned_gate == dr_fp.gate_nr) {

      //pushJungleGateDetection();


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

  filteredX = dr_state.x;
  filteredY = dr_state.y;
  return;
}


int transfer_measurement_local_2_global(float *_mx, float *_my, float dx, float dy)
{
  int i, j;
  float min_distance = 9999;

  dr_state.assigned_gate_index = -1;

  for (i = 0; i < MAX_GATES; i++) {
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
  }

  if(dr_state.assigned_gate_index == -1) {
    dr_state.assigned_gate_index = dr_fp.gate_nr;
  }

  return dr_state.assigned_gate_index;
}

void pushJungleGateDetection(void)
{
  if (gates[dr_fp.gate_nr].type == JUNGLE && jungleGate.flagJungleGateDetected == false
      && jungleGate.numJungleGateDetection < MAX_DETECTION) {
    jungleGate.jungleGateDetectionZ[jungleGate.numJungleGateDetection] = dr_vision.dz;
    jungleGate.jungleGateDetectionY[jungleGate.numJungleGateDetection] = dr_vision.dy;
    jungleGate.sumJungleGateHeight += dr_vision.dz;
    jungleGate.numJungleGateDetection++;
    jungleGate.jungleGateHeight = jungleGate.sumJungleGateHeight / jungleGate.numJungleGateDetection;
    if (jungleGate.numJungleGateDetection == MAX_DETECTION) {
      jungleGate.flagJungleGateDetected = true;
      if (jungleGate.jungleGateHeight > 0.0) {
        flagHighOrLowGate = UPPER_GATE;
      } else {
        flagHighOrLowGate = LOWER_GATE;
      }
    }
  }
}


int get_time_stamp()
{
  struct timeval te;
  gettimeofday(&te, NULL); // get current time
  long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
  int timeStamp = milliseconds%100000;
  //printf("Timestamp: %d\n",timeStamp);
  return timeStamp;
}


void calibrate_detection(float *mx,float *my)
{
    float k0_x = -0.0;
    float k1_x = 0.0;
    float k2_x = -0.0;

    float k0_y = 0.0;
    float k1_y = 0.0;
    float k2_y = -0.0;

    float x = *mx;
    float y = *my;

    *mx = x + k0_x + k1_x*x+k2_x*x*x;
    *my = y + k0_y + k1_y*y+k2_y*y*y;

}

void calibrate_ahrs_init()
{
    cali_ahrs.sum_bias_north = 0.0;
    cali_ahrs.sum_bias_east = 0.0;
    cali_ahrs.counter= 0;
    cali_ahrs.is_ahrs_calibrated = 0;
}

void calibrate_ahrs()
{
	int counter_start = 1000;
	int counter_end = 2000;

	if(cali_ahrs.counter<counter_start)
	{
		if(cali_ahrs.counter%10==0)
			printf("Calibrating AHRS [%.1f %%]\n",0.0);
	}

	if(cali_ahrs.counter > counter_start && cali_ahrs.counter < counter_end)
	{
		cali_ahrs.sum_bias_east += stateGetNedToBodyEulers_f()->phi;
		cali_ahrs.sum_bias_north += stateGetNedToBodyEulers_f()->theta;
		if(cali_ahrs.counter%10==0)
			printf("Calibrating AHRS [%.1f %%]\n",((float)cali_ahrs.counter-(float)counter_start)/(counter_end-counter_start)*100.0);
	}
	if(cali_ahrs.counter > counter_end)
	{
		cali_ahrs.bias_north = cali_ahrs.sum_bias_north/(counter_end-counter_start);
		cali_ahrs.bias_east = cali_ahrs.sum_bias_east/(counter_end-counter_start);
		cali_ahrs.is_ahrs_calibrated = 1;
		printf("ahrs calibration is done\n");
	}
	cali_ahrs.counter ++;
}
