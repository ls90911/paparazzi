/*
 * Copyright (C) Bart Slinger
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
 * @file "modules/sensors/ranger_through_datalink.c"
 * @author Bart Slinger
 * gets ranger data through datalink
 */

#include "modules/sensors/ranger_through_datalink.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/imu.h"
#include "state.h"
#include "abi_messages.h"

struct OrientationReps imu_to_mag;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void ranger_through_datalink_raw_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_RANGER(trans, dev, AC_ID, &ahrs.agl);
}
#endif

void ranger_through_datalink_init() {

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RANGER, ranger_through_datalink_raw_downlink);
#endif
}

void ranger_through_datalink_parse_msg() {
  uint32_t now_ts = get_sys_time_usec();
  struct Int32Vect3 raw_mag;
  /* (Ab)used SONAR message for this */
  raw_range = DL_SONAR_sonar_meas(dl_buffer);

  /* Rotate the Ranger */
  struct Int32RMat *imu_to_ranger_rmat = orientationGetRMat_i(&imu_to_ranger);
  int32_rmat_vmult(&imu.ranger_unscaled, imu_to_ranger_rmat, &raw_range);

  // Send and set the magneto IMU data
  imu_scale_mag(&imu);
  AbiSendMsgIMU_RANGER_INT16(RANGER_SENDER_ID, now_ts, &imu.range);
}