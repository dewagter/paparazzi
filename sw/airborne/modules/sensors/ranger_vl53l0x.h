/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/sensors/ranger_vl53l0x.h"
 * @author C. De Wagter
 * Sensor driver for VL53L0X laser range sensor
 */

#ifndef RANGER_VL53L0X_H
#define RANGER_VL53L0X_H

extern uint16_t vl53l0x_range;

extern void ranger_vl53_init(void);
extern void ranger_vl53_periodic(void);
extern void ranger_vl53_event(void);

#endif

