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
 * @file "modules/sonar/sonar_datalink.h"
 * @author OpenUAS
 * Driver for a sonar that gives distance data over the datalink
 */

#ifndef SONAR_DATALINK_H
#define SONAR_DATALINK_H

#include "std.h"

struct Sonar_Datalink {
  uint16_t meas; ///< Measured distance in mm
  float distance; ///< Scaled distance in m
  float offset; ///< Offset in m
  bool update_agl;
  bool compensate_rotation;
};

extern struct Sonar_Datalink sonar_datalink;

extern void sonar_datalink_init(void);
extern void sonar_datalink_periodic(void);
extern void sonar_datalink_downlink(void);

#endif /* SONAR_DATALINK_H */