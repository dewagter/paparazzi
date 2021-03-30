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
 */

// Own header
#include "modules/computer_vision/cv_cyberzoo_carpet.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef CYBERZOO_CARPET_FPS1
#define CYBERZOO_CARPET_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min1 = 0;

bool cod_draw1 = false;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters;


uint32_t find_carpet(struct image_t *img, bool draw);

struct image_t result;



/*
 * object_detector
 * @param img - input image to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img)
{
  bool draw = cod_draw1;

  // Filter and find centroid
  uint32_t count = find_carpet(img, draw);
  count ++;
  //VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);

  pthread_mutex_lock(&mutex);
  global_filters.color_count = 1;
  global_filters.x_c = 0;
  global_filters.y_c = 0;
  global_filters.updated = true;
  pthread_mutex_unlock(&mutex);

  result.ts = img->ts;      ///< The timestamp of creation
  result.eulers = img->eulers;   ///< Euler Angles at time of image
  result.pprz_ts = img->pprz_ts;       ///< The timestamp in us since system startup
  result.buf_idx = img->buf_idx;        ///< Buffer index for V4L2 freeing


  return &result;
}


void cyberzoo_carpet_init(void)
{
  image_create(&result, 240, 260, IMAGE_YUV422);

  memset(&global_filters, 0, sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);

#ifdef CYBERZOO_CARPET_LUM_MIN1
  cod_lum_min1 = CYBERZOO_CARPET_LUM_MIN1;
#endif
#ifdef CYBERZOO_CARPET_DRAW1
  cod_draw1 = CYBERZOO_CARPET_DRAW1;
#endif

  cv_add_to_device(&CYBERZOO_CARPET_CAMERA1, object_detector, CYBERZOO_CARPET_FPS1);

}

/*
 * find_carpet
 *
 * Finds the Cyberzoo Carpet
 *
 * @param img - input image to process formatted as YUV422.
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
uint32_t find_carpet(struct image_t *img, bool draw)
{
  uint32_t cnt = 0;
  uint8_t *buffer = img->buf;

  uint8_t *res = result.buf;

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y += 2) {
    //uint16_t dy = y * 2 * img->w;
    for (uint16_t x = 0; x < img->w; x += 2) {
      // Check if the color is inside the specified values
      uint8_t *yp1, *up, *vp, *yp2;
      // Even x
      up  = buffer++;      // U
      yp1 = buffer++;  // Y1
      vp  = buffer++;  // V
      yp2 = buffer++;
      /*
      if ((*yp1 > 220) || (*yp1 < 30)) {
        *yp1 = 0;
      }
      */

      uint16_t Y = (*yp1 << 6);
      int16_t  V = (*vp - 128);
      int16_t  U = (*up - 128);

      uint16_t R = (Y + 90 * V);
      uint16_t G = (Y - 22 * U - 45 * V);
      uint16_t B = (Y + 113 * U);

      if (R > (255 << 6)) R = (255 << 6);
      if (G > (255 << 6)) G = (255 << 6);
      if (B > (255 << 6)) B = (255 << 6);

      R = R >> 6;
      G = G >> 6;
      B = B >> 6;

      uint8_t Vmax = R;
      uint8_t Vmin = R;

      if ( G > Vmax) Vmax = G;
      if ( B > Vmax) Vmax = B;
      if ( G < Vmin) Vmin = G;
      if ( B < Vmin) Vmin = B;

      uint8_t C = Vmax - Vmin;
      uint8_t H = 0;

      //uint16_t temp = C;
      //temp = temp << 8;
      if (Vmax < 1) Vmax = 1;
      uint8_t S = (((uint16_t)C) << 7) / Vmax;

      if (C == 0) {
      } else if (R == Vmax) {
        H = (((int16_t)(G-B)) << 8) / C;
      } else if (G == Vmax) {
        H = (((int16_t)(B-R)) << 8) / C;
      } else if (B == Vmax) {
        H = (((int16_t)(R-G)) << 8) / C;
      }

      if (C >= 0) {

//      if ( 1 ) {
        if (draw){
          cnt++;
          //*yp = y % 255;  // make pixel brighter in image
          //*vp = x % 255;  // make pixel brighter in image
          //*up = y % 255;  // make pixel brighter in image
        }

if (draw){
        *res++ = *up;
        *res++ = *yp1;
        *res++ = *vp;
        *res++ = *yp2;
} else {
        *res++ = 127;
        *res++ = S;
        *res++ = 127;
        *res++ = S;
}

      }
    }
    buffer += 2 * img->w;
  }
  return cnt;
}

void cyberzoo_carpet_periodic(void)
{
  static struct color_object_t local_filters;
  pthread_mutex_lock(&mutex);
  memcpy(&local_filters, &global_filters, sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters.updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters.x_c, local_filters.y_c,
        0, 0, local_filters.color_count, 0);
    local_filters.updated = false;
  }
}
