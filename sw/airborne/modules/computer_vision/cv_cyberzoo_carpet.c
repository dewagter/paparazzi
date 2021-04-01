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


float find_carpet(struct image_t *img, bool draw);

//struct image_t result;

volatile float carpet_land_direction = 0;
volatile float carpet_land_certainty = 0;
volatile float carpet_land_colliding = 0;


/*
 * object_detector
 * @param img - input image to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img)
{
  bool draw = cod_draw1;

  // Filter and find centroid
  find_carpet(img, draw);
  //VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);

  pthread_mutex_lock(&mutex);
  global_filters.color_count = 1;
  global_filters.x_c = 0;
  global_filters.y_c = 0;
  global_filters.updated = true;
  pthread_mutex_unlock(&mutex);

  /*
  result.ts = img->ts;      ///< The timestamp of creation
  result.eulers = img->eulers;   ///< Euler Angles at time of image
  result.pprz_ts = img->pprz_ts;       ///< The timestamp in us since system startup
  result.buf_idx = img->buf_idx;        ///< Buffer index for V4L2 freeing
  */
  return img; //&result;
}


void cyberzoo_carpet_init(void)
{
  //image_create(&result, 240, 260, IMAGE_YUV422);

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


uint8_t HSV(uint8_t* yp, uint8_t* up, uint8_t* vp);
uint8_t HSV(uint8_t* yp, uint8_t* up, uint8_t* vp) {

  uint16_t Y = (*yp << 6);
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

  return S + H;
}


static inline uint8_t tree(uint8_t Y, uint8_t U, uint8_t V ) {
  if (U <= 111) {
    if (V <= 143) {
      if (Y <= 199) {
        return 255;
      } else { // Y > 199
        return 0;
      }
    } else { // V > 143
      if (V <= 146) {
        return 0;
      } else { // V > 146
        return 0;
      }
    }
  } else { // U > 111
    if (U <= 115) {
      if (V <= 132) {
        return 255;
      } else { // V > 132
        return 0;
      }
    } else { // U > 115
      if (U <= 117) {
        return 0;
      } else { // U > 117
        return 0;
      }
    }
  }
}


inline static void stay_away_from_edges(int16_t *confidence, int size) {
    for (int i=1;i < size - 1;i++) {

    if (confidence[i-1] <= 1) {
      confidence[i] = confidence[i] / 2;
    }
    if (confidence[i+1] <= 1) {
      confidence[i] = confidence[i] / 2;
    }

    // Jumps
    int16_t diff = confidence[i-1] - confidence[i];
    if (diff < 0) diff = -diff;

    if (diff > 20) {
      confidence[i] = confidence[i] / 2;
    };

    diff = confidence[i] - confidence[i+1];
    if (diff < 0) diff = -diff;

    if (diff > 20) {
      confidence[i] = confidence[i] / 2;
    };

  }

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

float find_carpet(struct image_t *img, bool draw)
{
  uint32_t cnt = 0;
  uint8_t *buffer = img->buf;

  ////////////////////////////////////////////////////
  // Discision Tree Classifier

  //uint8_t *res = result.buf;
  uint8_t *max = img->buf;
  max += img->buf_size;

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y += 1) {
    //uint16_t dy = y * 2 * img->w;
    for (uint16_t x = 0; x < img->w; x += 2) {
      // Check if the color is inside the specified values
      uint8_t *yp1, *up, *vp; //, *yp2;

      // Even x
      up  = buffer++;  // U
      yp1 = buffer++;  // Y1
      vp  = buffer++;  // V
      //yp2 = 
      buffer++;  // Y2

      if (buffer > max) {
        fprintf(stderr,"[carpetland] P=(%d,%d) OUT_OF_BUFFER\n", x,y);
        return cnt;
      }

      // Carpet
      uint8_t g = tree(*yp1, *up, * vp);

      if (draw) {
        if (g > 0) {
          *vp = 0;
        }
      }
    }
    //buffer += 2 * img->w;
  }

  ////////////////////////////////////////////////////
  // Bottom up grass search

  // Vector of grass
  #define MAX_VEC 52
  uint8_t nr_of_vec = 0;
  int16_t vec[MAX_VEC];

  int8_t i = 0;
  uint16_t h2 = img->w / 2;
  uint16_t row_offset = img->w * 2;
  for (uint16_t y = 10; y < img->h; y += 10) {
    // Row offset
    buffer = img->buf;
    buffer += y * row_offset;
    // Carpet byte offset

    uint16_t height = 0;
    uint16_t cnt_col = 0;
    uint16_t cnt_none = 0;
    for (uint16_t x = 0; x < h2; x+=2 ) {
      if (buffer > max) {
        fprintf(stderr,"[carpetland] P=(%d,%d) OUT_OF_BUFFER 2\n", x,y);
        return 0;
      }
      uint8_t m = buffer[2];


      // If grass
      if (m == 0) {
        cnt_none = 0;
        cnt_col ++;
        // Check pixels next to this one
        if (buffer[2 + row_offset] == 0) {
          cnt ++;
        }
      } else {
        cnt_none ++;
        if (cnt_none > 25) {
          break;
        }
        cnt_col = 0;
      }
      if (cnt_col > 5) {
        height = x;
      }

      buffer += 4;
    }
    // Store height of grass
    vec[i] = height;
    i++;
    // Store how many vecs we have
    nr_of_vec = i;
    if (i > MAX_VEC) {
      fprintf(stderr,"[carpetland] i=(%d) OUT_OF_BUFFER 3\n", i);
      break;
    }
    // Draw line
    buffer = img->buf;
    buffer += y * img->w * 2;
    for (int16_t x=0;x<height;x++) {
      //buffer[1] = 255;
      //buffer[1+row_offset] = 255;
      buffer += 2;
    }
  }

  ////////////////////////////////////////////////////
  // Confidence Search
  
  int16_t confidence[MAX_VEC];
  for (i=0;i < MAX_VEC;i++) {
    confidence[i]=0;
  }
  
  for (i=2;(i < nr_of_vec-2) && (i < MAX_VEC-2);i++) {
    // Start with a copy
    confidence[i] = vec[i];

    if (vec[i-1] <= 1) {
      confidence[i] = confidence[i] / 2;
    }
    if (vec[i-2] <= 1) {
      confidence[i] = confidence[i] / 2;
    }
    if (vec[i+1] <= 1) {
      confidence[i] = confidence[i] / 2;
    }
    if (vec[i+2] <= 1) {
      confidence[i] = confidence[i] / 2;
    }

    // Jumps
    int16_t diff = vec[i-2] - vec[i];
    if (diff < 0) diff = -diff;

    if (diff > 20) {
      confidence[i] = confidence[i] / 2;
    };
     // Jumps
    diff = vec[i] - vec[i+2];
    if (diff < 0) diff = -diff;

    if (diff > 20) {
      confidence[i] = confidence[i] / 2;
    };
  }

  //stay_away_from_edges(confidence, nr_of_vec);
  //stay_away_from_edges(confidence, nr_of_vec);

  ////////////////////////////////////////////////////
  // Search Maximum with moving window
  int8_t best_direction = 0;
  int16_t best_score = 0;
  for (i=4;(i < nr_of_vec-4) && (i < MAX_VEC-4);i++) {
    // Start with a copy
    int16_t sum =  confidence[i-4]
                 + confidence[i-3]
                 + confidence[i-2]
                 + confidence[i-1]
                 + confidence[i]
                 + confidence[i+1]
                 + confidence[i+2]
                 + confidence[i+3]
                 + confidence[i+4];
    if (sum > best_score) {
      best_score = sum;
      best_direction = i;
    }
  }

  int16_t total = 0;
  for (i=26-16;i<26+16;i++) {
    total += confidence[i];
  }
  carpet_land_colliding = ((float)total) / 32.0f;

  // Draw confidence line
  i = 0;
  for (uint16_t y = 10+1; y < img->h; y += 10) {
    // Row offset
    buffer = img->buf;
    buffer += y * row_offset;
    // Draw line
    buffer = img->buf;
    buffer += y * img->w * 2;
    for (int16_t x=0;x<confidence[i];x++) {
      buffer[1] = 255;
      if (i == best_direction) {
        buffer[0] = 255;
        buffer[1] = 255;
        buffer[0+row_offset] = 255;
        buffer[1+row_offset] = 255;
        buffer[0-row_offset] = 255;
        buffer[1-row_offset] = 255;
      }
      buffer += 2;
    }
    i++;
  }

  // Scale direction from -1 -> 1 and export
  float hnvf = ((float) nr_of_vec) / 2.0f;
  float dir = ((float) best_direction - hnvf) / hnvf;
  carpet_land_direction = dir;
  carpet_land_certainty = ((float)best_score);
  return 0;
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
