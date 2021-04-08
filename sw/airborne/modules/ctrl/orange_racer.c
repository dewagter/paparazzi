/*
 * Copyright (C) 2015
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/orange_racer.h
 * @brief example empty controller
 *
 */

#include "modules/ctrl/orange_racer.h"
#include "state.h"
//#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
//#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "autopilot.h"

#include "modules/computer_vision/cv_cyberzoo_carpet.h"

#include <stdio.h>

// Own Variables

struct ctrl_module_demo_struct {
  // Time counter
  float time;

  float heading;


  float integralx;
  float integraly;


  // Output command
  struct Int32Eulers cmd;

} ctrl;

// Settings
//float comode_time = 0;


////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  ctrl.time = 0;
  ctrl.heading = 0;

  fprintf(stdout,"[orange_racer] INIT\n");
}

void guidance_h_module_enter(void)
{
  fprintf(stdout,"[orange_racer] ENTER\n");
  // Store current heading
  ctrl.heading = stateGetNedToBodyEulers_f()->psi;

  ctrl.integralx = 0;
  ctrl.integraly = 0;

  ctrl.time = 0;

  // Convert RC to setpoint
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

void guidance_h_module_read_rc(void)
{
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

float race_multiplier = 0.25;
float race_radius = 3;

float orange_racer_K_V_P = 0.3;


void guidance_h_module_run(bool in_flight)
{

  // Import state
  //float phi = stateGetNedToBodyEulers_f()->phi;
  //float theta = stateGetNedToBodyEulers_f()->theta;
  float psi = stateGetNedToBodyEulers_f()->psi;

  struct NedCoor_f* x = stateGetPositionNed_f();
  struct NedCoor_f* v = stateGetSpeedNed_f();


  //static float accelerator = 1.0;
  //fprintf(stdout,"[orange_racer] RUN: dir = %f cert  =%f\n",  carpet_land_direction, carpet_land_certainty);
  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);

  float multiplier = race_multiplier;


  float dist_center = sqrt(x->x*x->x + x->y*x->y);

  /*

  static float filtx = 0;
  static float filty = 0;

  float bearing = atan2(x->y, x->x);
  float dhead = bearing - psi;

  float yaw_gain = 0.5;

  if (dhead < 0) dhead = -dhead;
  dhead = DegOfRad(dhead);
 
  fprintf(stdout,"[orange_racer] d=%f  bear = %f psi = %f   dg=%f \n",  dist_center, bearing, psi, dhead);

  float certainty = carpet_land_certainty;

  float danger = carpet_land_colliding;

  if (danger < 9) {
    certainty = 10;    
  }

  //float yawrate = 

  if (dist_center > 3.7) {
    if (dhead < 110)
      certainty = 39;
  } else if (dist_center > 2.5) {
    if (dhead < 110)
      certainty = 79;
  }

  float speedx = 1.0; // certainty / 500.0 * 0.5;
  if (certainty <  50) {
    speedx = 0.5;
    //ctrl.heading += 0.002 * yaw_gain;
  } else  if (certainty <  100) {
    speedx = 0.75;
    //ctrl.heading += (0.004 + carpet_land_direction * 0.0008) * multiplier * yaw_gain;
  } else {
    //ctrl.heading += (carpet_land_direction * 0.0025) * multiplier * yaw_gain;
  }
  float speedy = 0; //speedx * carpet_land_direction / 1.5;

  if (danger < 9) {
    certainty = 10;    
    speedx = -0.05;
  }

  speedx *= multiplier;


  filtx += (speedx - filtx) / 30;
  filty += (speedy - filty) / 30;



  //fprintf(stdout,"[orange_racer]  v_s = %f, %f psi = %f\n",  speedx, speedy, ctrl.heading);
*/

  // GO

  //ctrl.time += 0.002 * multiplier;
  //accelerator -= 0.00002;
  //if (accelerator < 0.1) accelerator = 0.1;

  // Flightplan
  float heading = ctrl.heading;

  float radius = race_radius;
  float vff = 0;//radius / 5;


  // Position setpoint
  float x_s = sin(heading) * radius;
  float y_s = -cos(heading) * radius;

  navigation_carrot.x = POS_BFP_OF_REAL( y_s);
  navigation_carrot.y = POS_BFP_OF_REAL(-x_s);

  VECT2_COPY(navigation_carrot, navigation_target);

  // Position error
  float dx = x_s - x->x;
  float dy = y_s - x->y;

  // Body frame
	float dx_b =  cos(psi)*dx + sin(psi)*dy;
	float dy_b = -sin(psi)*dx + cos(psi)*dy;
  
  // Position Loop (Body)
  #define X_K_P 0.4
  float vx_s_b = ( dx_b ) * X_K_P + vff;
  float vy_s_b = ( dy_b ) * X_K_P;

  //vx_s_b = filtx;
  //vy_s_b = filty;


  float vtot = sqrt(v->x*v->x + v->y*v->y);

  float yawrate = 6.28 / vtot;

  float d_g = (dist_center - radius) / 2 + 1;

  yawrate *= d_g;


  ctrl.heading += yawrate / 512.0f;

  // Global velociy
	//float vx_b =  cos(psi) * v->x + sin(psi) * v->y;
	//float vy_b = -sin(psi) * v->x + cos(psi) * v->y;


  // Speed loop
  //float dvx = vx_s_b - vx_b;
  //float dvy = vy_s_b - vy_b;

  //BoundAbs(dvx,5);
  //BoundAbs(dvy,5);

  // 	// Coordinated turn
	float new_roll = -atan(vtot / 9.81 * yawrate);

  static float lastdvx = 0;
  static float lastdvy = 0;


  #define V_K_FF 0.00
  //#define V_K_P  0.3
  #define V_K_D  0.0
  #define V_K_I  0.0

  //ctrl.integralx += dvx * V_K_I;
  //ctrl.integraly += dvy * V_K_I;

  float roll = new_roll; //dvy * orange_racer_K_V_P    + V_K_FF * vy_s_b + V_K_D * (lastdvy - dvy) + ctrl.integraly + new_roll;
  float pitch = - multiplier; //- dvx * orange_racer_K_V_P - V_K_FF * vx_s_b - V_K_D * (lastdvx - dvx) - ctrl.integralx;

  //lastdvx = dvx;
  //lastdvy = dvy;

  //fprintf(stderr, "Vtot=(%f), psi = %f, dv=(%f,%f)\n", vtot, psi, dvx, dvy  );


  BoundAbs(roll, RadOfDeg(35));
  BoundAbs(pitch,RadOfDeg(35));

  //static heading_s = 0;
  float track = atan2(v->y,v->x);


  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll);
  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch);
  ctrl.cmd.psi = ANGLE_BFP_OF_REAL(track);

  stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
  stabilization_attitude_run(in_flight);
/**/
  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}

