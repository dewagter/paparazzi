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
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "autopilot.h"

#include <stdio.h>

// Own Variables

struct ctrl_module_demo_struct {
// RC Inputs
  struct Int32Eulers rc_sp;

// Output command
  struct Int32Eulers cmd;

} ctrl;


// Settings
float comode_time = 0;


////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
}

void guidance_h_module_enter(void)
{
  // Store current heading
  ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  // Convert RC to setpoint
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

void guidance_h_module_read_rc(void)
{
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}


float time = 0;

void guidance_h_module_run(bool in_flight)
{
  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);

  time += 0.002;

  // Import state
  float phi = stateGetNedToBodyEulers_f()->phi;
  float theta = stateGetNedToBodyEulers_f()->theta;
  float psi = stateGetNedToBodyEulers_f()->psi;

  struct NedCoor_f* x = stateGetPositionNed_f();
  struct NedCoor_f* v = stateGetSpeedNed_f();

  // Flightplan
  float heading = time / 2;

  float x_s = sin(heading) * 4;
  float y_s = -cos(heading) * 4;

  float dx = x_s - x->x;
  float dy = y_s - x->y;

  // Body frame
	double dx_b =  cos(psi)*dx + sin(psi)*dy;
	double dy_b = -sin(psi)*dx + cos(psi)*dy;


  // Position Loop (Body)
  #define X_K_P 0.5
  float vx_s_b = ( dx_b ) * X_K_P;
  float vy_s_b = ( dy_b ) * X_K_P;

  // Global velociy
	double vx_b =  cos(psi) * v->x + sin(psi) * v->y;
	double vy_b = -sin(psi) * v->x + cos(psi) * v->y;

  // Speed loop
  float dvx = vx_s_b - vx_b;
  float dvy = vy_s_b - vy_b;

  // 	// Coordinated turn
	// new_roll += -atan(vel_x_est_velFrame / 9.81 * this->yaw_rate_cmd);


#define V_K_FF 0.02
#define V_K_P  0.9

  float roll = dvy * V_K_P + V_K_FF * vy_s_b;
  float pitch = - dvx * V_K_P - V_K_FF * vx_s_b;

  fprintf(stderr, "VX = (%f,%f), psi = %f, dv=(%f,%f)\n", v->x, v->y, psi, dvx, dvy  );


  BoundAbs(roll,RadOfDeg(35));
  BoundAbs(pitch,RadOfDeg(35));

  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll);
  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch);
  ctrl.cmd.psi = ANGLE_BFP_OF_REAL(heading);

  stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
  stabilization_attitude_run(in_flight);

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}

