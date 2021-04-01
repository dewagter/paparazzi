/*
 * Copyright (C) 2021 C. De Wagter
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_cyberzoo_carpet.h
 * Specific color filter to find green carpet
 */

#ifndef CYBERZOO_CARPET_CV_H
#define CYBERZOO_CARPET_CV_H

#include <stdint.h>
#include <stdbool.h>

// Module settings
extern uint8_t cod_lum_min1;

extern bool cod_draw1;

// Module output
extern volatile float carpet_land_direction;
extern volatile float carpet_land_certainty;
extern volatile float carpet_land_colliding;

// Module functions
extern void cyberzoo_carpet_init(void);
extern void cyberzoo_carpet_periodic(void);

#endif /* CYBERZOO_CARPET_CV_H */
