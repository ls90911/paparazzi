/*
 * Copyright (C) OpenUAS
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
 * @file "modules/sonar/sonar_datalink.c"
 * @author OpenUAS
 * Driver for an sonar that get data via datalink
 * In some case an 3rd party device that injects sonar measurments into thew PPRZ messages stream
 */

//FIXME: Better to start using IMCU_REMOTE_GROUND, message is more representative

#include "subsystems/abi.h"               //for messages subscription
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/ins/vf_extended_float.h"
#include "subsystems/ins/ins_int.h"

#include "pprzlink/messages.h"
#include "pprzlink/pprz_transport.h"
#include "message_pragmas.h"

#include "state.h"                        //For rotation compensation calc

#include "subsystems/abi.h"                 // rssi messages subscription
#include "generated/airframe.h"           // For AC_ID

#include "modules/sonar/sonar_datalink.h"

// Add an offset to the measurments (in meters)
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 0.f
#endif

// Send AGL data over ABI so it can be used in state if needed
#ifndef USE_SONAR_DATALINK_AGL
#define USE_SONAR_DATALINK_AGL 0
#endif

// Rotation compensation
#ifndef SONAR_DATALINK_COMPENSATE_ROTATION
#define SONAR_DATALINK_COMPENSATE_ROTATION 0
#endif

struct Sonar_Datalink sonar_datalink;
//struct MedianFilterInt sonar_datalink_filter;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/**
 * Message sonar
 */
static void sonar_datalink_send_sonar(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SONAR(trans, dev, AC_ID,
                      &sonar_datalink.meas,
                      &sonar_datalink.distance);
}

#endif

/**
 * Downlink message for debug
 */
void sonar_datalink_downlink(void)
{
#ifdef SENSOR_SYNC_SEND_SONAR
  // Send Telemetry report for debugging
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_datalink.meas, &sonar_datalink.distance);
#endif
}

/**
 * Initialization function
 */
void sonar_datalink_init(void)
{
  sonar_datalink.meas = 0;//todo add minimum of 30cm as per sensor specs?
  sonar_datalink.distance = 0.f;//not max?
  sonar_datalink.offset = SONAR_OFFSET;
  sonar_datalink.update_agl = true;//USE_SONAR_DATALINK_AGL;
  sonar_datalink.compensate_rotation = true;//SONAR_DATALINK_COMPENSATE_ROTATION;

 // init_median_filter_i(&sonar_datalink_filter, SONAR_DATALINK_MEDIAN_LENGTH);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONAR, sonar_datalink_send_sonar);
#endif
}

void sonar_datalink_periodic(void)
{
#ifndef SITL

  uint8_t sender_id = SenderIdOfPprzMsg(dl_buffer);
  uint8_t msg_id = IdOfPprzMsg(dl_buffer);

  //sonar_datalink.meas = (uint16_t)msg_id; //FOR TEMP DEBUGGING ONLY to validate if message ID of SONAR (236 in our case) is there
  //sonar_datalink.meas = 45; //FOR TEMP DEBUGGING ONLY

  // Data needs to come from self AC
  //if (sender_id == AC_ID){
    if (msg_id==DL_SONAR) { //newer is PPRZ_MSG_ID_SONAR
      sonar_datalink.meas = DL_SONAR_sonar_meas(dl_buffer);
      //sonar_datalink.meas = 49; //FOR TEMP DEBUGGING ONLY
      sonar_datalink.distance = (float)(sonar_datalink.meas - sonar_datalink.offset);// * SONAR_SCALE;
      // check if data is within range TODO: test all cases with offset
      if (sonar_datalink.meas > 0) {
        sonar_datalink.distance = (float)sonar_datalink.meas / 1000.f + sonar_datalink.offset;

        //sonar_datalink.meas = update_median_filter_i( &sonar_datalink_filter, (uint32_t)(sonar_datalink.meas));//FIXME: if you need it

        // Compensate AGL measurement for body rotation
        if (sonar_datalink.compensate_rotation) {
          float phi = stateGetNedToBodyEulers_f()->phi;
          float theta = stateGetNedToBodyEulers_f()->theta;
          float gain = (float)fabs((double)(cosf(phi) * cosf(theta)));
          sonar_datalink.distance = sonar_datalink.distance * gain;
        }

        // send ABI message if requested flag set to true so yeah dynamic runtime o set or not may come in handy
        //sonar_datalink.distance=.66f; //FOR TEMP DEBUGGING ONLY
        //if (sonar_datalink.update_agl) {
        uint32_t now_ts = get_sys_time_usec();
        //sonar_datalink.distance=.77f; //FOR TEMP DEBUGGING ONLY
        vff_update_agl(-sonar_datalink.distance, 0.2); //FOR TEMP DEBUGGING ONLY
        ins_int.propagation_cnt = 0; //FOR TEMP DEBUGGING ONLY
        //AbiSendMsgAGL(AGL_SONAR_ADC_ID, now_ts, sonar_datalink.distance);//AGL_SONAR_SONAR_DATALINK_ID
        //}
      }
    }
 // }
#else // SITL
  sonar_datalink.distance = stateGetPositionEnu_f()->z;//not correct, no rotateion compensation and not really AGL
  Bound(sonar_datalink.distance, 0.1f, 7.0f);//not definable  fixed at 7 ?
#endif // SITL

}