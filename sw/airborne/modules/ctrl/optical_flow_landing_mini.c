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
 * @file modules/ctrl/optical_flow_landing.h
 * @brief This module implements optical flow landings in which the divergence is kept constant.
 * When using a fixed gain for control, the covariance between thrust and divergence is tracked,
 * so that the drone knows when it has arrived close to the landing surface. Then, a final landing
 * procedure is triggered. It can also be set to adaptive gain control, where the goal is to continuously
 * gauge the distance to the landing surface. In this mode, the drone will oscillate all the way down to
 * the surface.
 *
 * de Croon, G.C.H.E. (2016). Monocular distance estimation with optical flow maneuvers and efference copies:
 * a stability-based strategy. Bioinspiration & biomimetics, 11(1), 016004.
 * <http://iopscience.iop.org/article/10.1088/1748-3190/11/1/016004>
 *
 */

#define DEBUG_PRINT(...) {}

#include "optical_flow_landing_mini.h"
#include "modules/computer_vision/textons.h"

float dt;
float divergence;
volatile float divergence_vision;  	///< value from stereoboard
float divergence_vision_dt;		///< divergence/dt
float normalized_thrust;


volatile uint8_t run_control = 0;


// variables for in message:
float pstate;
float pused;
volatile int has_new_vision_message;
int landing;
float previous_err;
float previous_cov_err;
float cov_div;


// arrays containing histories for determining covariance
float thrust_history[COV_WINDOW_SIZE];
float divergence_history[COV_WINDOW_SIZE];
float past_divergence_history[COV_WINDOW_SIZE];
unsigned long ind_hist;
// SSL:
float *text_dists[MAX_SAMPLES_LEARNING];
float sonar[MAX_SAMPLES_LEARNING];
float gains[MAX_SAMPLES_LEARNING];
float cov_divs_log[MAX_SAMPLES_LEARNING];
float weights[TEXTONS_N_TEXTONS + 1];


// minimum value of the P-gain for divergence control
// adaptive control will not be able to go lower
#define MINIMUM_GAIN 0.1

// SSL: we will learn unstable gains, but need stable gains for landing
// this factor represents the trade-off between stability and performance
// 1.0 = unstable, 0.0 = no performance
#define STABLE_GAIN_FACTOR 0.5

// used for automated landing:
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/navigation/common_flight_plan.h"

// ************************
// include textons for SSL:
// ************************
//#include <stdio.h>
float last_texton_distribution[TEXTONS_N_TEXTONS];
float texton_distribution_stereoboard[TEXTONS_N_TEXTONS];



#include "subsystems/datalink/telemetry.h"

// sending the divergence message to the ground station:
static void send_divergence(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIVERGENCE(trans, dev, AC_ID,
                           &divergence, &divergence_vision_dt, &normalized_thrust,
                           &cov_div, &pstate, &pused, &(of_landing_ctrl.agl));
}


// sending the divergence message to the ground station:
static void send_training(struct transport_tx *trans, struct link_device *dev)
{
  static int cnt = 0;
  float cov_dif_send = cov_div;
  float gain_send = pstate;
  float extra_send = cnt++;

  pprz_msg_send_TRAININGDATA(trans, dev, AC_ID,
      &texton_distribution_stereoboard[0], &texton_distribution_stereoboard[1], &texton_distribution_stereoboard[2],
      &texton_distribution_stereoboard[3], &texton_distribution_stereoboard[4], &texton_distribution_stereoboard[5],
      &texton_distribution_stereoboard[6], &texton_distribution_stereoboard[7], &texton_distribution_stereoboard[8], &texton_distribution_stereoboard[9],
      &cov_dif_send, &gain_send, &extra_send);

}


#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"

// horizontal control:
#include "firmwares/rotorcraft/guidance/guidance_h.h"


unsigned int n_read_samples;

// paparazzi files for doing svd etc.:
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.h"
#include "math/pprz_simple_matrix.h"

/* Default sonar/agl to use */
#ifndef OPTICAL_FLOW_LANDING_AGL_ID
#define OPTICAL_FLOW_LANDING_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OPTICAL_FLOW_LANDING_AGL_ID)

/* Use optical flow estimates */
#ifndef OPTICAL_FLOW_LANDING_OPTICAL_FLOW_ID
#define OPTICAL_FLOW_LANDING_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OPTICAL_FLOW_LANDING_OPTICAL_FLOW_ID)

/* Textons from the stereoboard */
#ifndef OPTICAL_FLOW_LANDING_TEXTONS_ID
#define OPTICAL_FLOW_LANDING_TEXTONS_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OPTICAL_FLOW_LANDING_TEXTONS_ID)


// Other default values:
#ifndef OPTICAL_FLOW_LANDING_PGAIN
#define OPTICAL_FLOW_LANDING_PGAIN 1.0
#endif

#ifndef OPTICAL_FLOW_LANDING_IGAIN
#define OPTICAL_FLOW_LANDING_IGAIN 0
#endif

#ifndef OPTICAL_FLOW_LANDING_DGAIN
#define OPTICAL_FLOW_LANDING_DGAIN 0.0
#endif

#ifndef OPTICAL_FLOW_LANDING_VISION_METHOD
#define OPTICAL_FLOW_LANDING_VISION_METHOD 1
#endif

#ifndef OPTICAL_FLOW_LANDING_CONTROL_METHOD
#define OPTICAL_FLOW_LANDING_CONTROL_METHOD 0
#endif

#ifndef OPTICAL_FLOW_LANDING_COV_METHOD
#define OPTICAL_FLOW_LANDING_COV_METHOD 0
#endif

// ABI callback handles
static abi_event agl_ev; ///< The altitude ABI event
static abi_event optical_flow_ev;
static abi_event textons_ev;

// ABI callback functions
/// Callback function of the ground altitude
static void vertical_ctrl_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);
// Callback function of the optical flow estimate:
static void vertical_ctrl_optical_flow_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp __attribute__((unused)), int16_t flow_x __attribute__((unused)), int16_t flow_y __attribute__((unused)),
    int16_t flow_der_x __attribute__((unused)), int16_t flow_der_y __attribute__((unused)),
    uint8_t quality __attribute__((unused)), float size_divergence, float dist __attribute__((unused)));
// Callback function of the texton histogram from the stereoboard:
static void vertical_ctrl_textons_cb(uint8_t sender_id __attribute__((unused)), uint8_t histogram0, uint8_t histogram1,
                                     uint8_t histogram2, uint8_t histogram3, uint8_t histogram4, uint8_t histogram5,
                                     uint8_t histogram6, uint8_t histogram7, uint8_t histogram8, uint8_t histogram9);



struct OpticalFlowLanding of_landing_ctrl;



// Module functions called from module
void optical_flow_landing_init(void)
{
  unsigned int i;

  of_landing_ctrl.agl = 0.0f;
  of_landing_ctrl.agl_lp = 0.0f;
  of_landing_ctrl.vel = 0.0f;
  of_landing_ctrl.divergence_setpoint = 0.0f;
  of_landing_ctrl.cov_set_point = -0.025f;
  of_landing_ctrl.cov_limit = 2.0f; // 0.0010f; //1.0f; // for cov(uz,div)
  of_landing_ctrl.lp_factor = 0.75f;
  of_landing_ctrl.pgain = OPTICAL_FLOW_LANDING_PGAIN;
  of_landing_ctrl.igain = OPTICAL_FLOW_LANDING_IGAIN;
  of_landing_ctrl.dgain = OPTICAL_FLOW_LANDING_DGAIN;
  of_landing_ctrl.sum_err = 0.0f;
  of_landing_ctrl.nominal_thrust = 0.81;
  of_landing_ctrl.VISION_METHOD = OPTICAL_FLOW_LANDING_VISION_METHOD;
  of_landing_ctrl.CONTROL_METHOD = OPTICAL_FLOW_LANDING_CONTROL_METHOD;
  of_landing_ctrl.COV_METHOD = OPTICAL_FLOW_LANDING_COV_METHOD;
  of_landing_ctrl.delay_steps = 40;
  of_landing_ctrl.pgain_adaptive = 10.0;
  of_landing_ctrl.igain_adaptive = 0.25;
  of_landing_ctrl.dgain_adaptive = 0.00;
  of_landing_ctrl.learn_gains = false;
  of_landing_ctrl.stable_gain_factor = STABLE_GAIN_FACTOR;
  of_landing_ctrl.load_weights = false;
  of_landing_ctrl.close_to_edge = 0.005;
  of_landing_ctrl.use_bias = false;
  of_landing_ctrl.snapshot = false;

  // clear histories:
  ind_hist = 0;
  for (i = 0; i < COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
  }

  // reset errors, thrust, divergence, etc.:
  previous_err = 0.0f;
  previous_cov_err = 0.0f;
  normalized_thrust = 0.0f;
  divergence = 0.0f;
  divergence_vision = 0.0f;
  divergence_vision_dt = 0.0f;
  cov_div = 0.0f;
  dt = 0.0f;
  pstate = of_landing_ctrl.pgain;
  pused = pstate;
  of_landing_ctrl.agl_lp = 0.0f;
  landing = 0;

  // SSL:
  for (i = 0; i < TEXTONS_N_TEXTONS; i++) {
    last_texton_distribution[i] = 0.0f;
  }
  for (i = 0; i <= TEXTONS_N_TEXTONS; i++) {
    weights[i] = 0.0f;
  }


  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(OPTICAL_FLOW_LANDING_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);
  // Subscribe to the optical flow estimator:
  AbiBindMsgOPTICAL_FLOW(OPTICAL_FLOW_LANDING_OPTICAL_FLOW_ID, &optical_flow_ev, vertical_ctrl_optical_flow_cb);
  // Subscribe to the textons estimator:
  AbiBindMsgTEXTONS(OPTICAL_FLOW_LANDING_TEXTONS_ID, &textons_ev, vertical_ctrl_textons_cb);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DIVERGENCE, send_divergence);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TRAININGDATA, send_training);
}

void optical_flow_landing_periodic(void)
{
  float div_factor; // factor that maps divergence in pixels as received from vision to /frame

  /***********
   * VISION
   ***********/

  if (of_landing_ctrl.VISION_METHOD == 0) {

    float lp_height; // low-pass height

    // SIMULATED DIVERGENCE:

    // USE OPTITRACK HEIGHT
    of_landing_ctrl.agl = (float) gps.lla_pos.alt / 1000.0f;
    // else we get an immediate jump in divergence when switching on.
    if (of_landing_ctrl.agl_lp < 1E-5 || ind_hist == 0) {
      of_landing_ctrl.agl_lp = of_landing_ctrl.agl;
    }
    if (fabs(of_landing_ctrl.agl - of_landing_ctrl.agl_lp) > 1.0f) {
      // ignore outliers:
      of_landing_ctrl.agl = of_landing_ctrl.agl_lp;
    }
    // calculate the new low-pass height and the velocity
    lp_height = of_landing_ctrl.agl_lp * of_landing_ctrl.lp_factor + of_landing_ctrl.agl *
                (1.0f - of_landing_ctrl.lp_factor);

    // only calculate velocity and divergence if dt is large enough:
    {
      of_landing_ctrl.vel = (lp_height - of_landing_ctrl.agl_lp) * 512.0;
      of_landing_ctrl.agl_lp = lp_height;

      run_control = 1;

      // calculate the fake divergence:
      if (of_landing_ctrl.agl_lp > 0.0001f) {
        divergence = of_landing_ctrl.vel / of_landing_ctrl.agl_lp;
        divergence_vision_dt = (divergence_vision / dt); // TODO: shouldn't this be divergence in this fake vision case?
        if (fabs(divergence_vision_dt) > 1E-5) {
          div_factor = divergence / divergence_vision_dt;
        }
      } else {
        divergence = 1000.0f;
        // perform no control with this value (keeping thrust the same)
        return;
      }
    }
  } else {
    // USE REAL VISION OUTPUTS:

    if (has_new_vision_message) { // (ind_hist > 1) {

      // TODO: this div_factor depends on the subpixel-factor (automatically adapt?)
      // div_factor = (vz / z) - from optitrack or similar, divided by (divergence_vision / dt)
      div_factor = 0.000432;
      divergence_vision_dt = (divergence_vision * 26.0) * div_factor; // delta_t);

      // low-pass filter the divergence:
      divergence = divergence * of_landing_ctrl.lp_factor + (divergence_vision_dt * (1.0f - of_landing_ctrl.lp_factor));
      has_new_vision_message = 0;

      run_control = 1;
    }
  }
}



/**
 * Reset all variables:
 */
void reset_all_vars(void)
{

  int i;
  of_landing_ctrl.sum_err = 0;
  stabilization_cmd[COMMAND_THRUST] = 0;
  ind_hist = 0;
  of_landing_ctrl.agl_lp = 0;
  cov_div = of_landing_ctrl.cov_set_point;
  normalized_thrust = 0.0f;
  dt = 0.0f;
  previous_err = 0.0f;
  previous_cov_err = 0.0f;
  divergence = of_landing_ctrl.divergence_setpoint;
  for (i = 0; i < COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
  }
  landing = 0;

  // SSL:
  for (i = 0; i < TEXTONS_N_TEXTONS; i++) {
    last_texton_distribution[i] = 0.0f;
  }
}

/**
 * Run the optical flow landing module
 */

void guidance_v_module_run(bool in_flight)
{

  dt = 1.0f / 512.0f ; //+= ((float)delta_t);// / 1000.0f;

  if (!in_flight) {


    // SSL: only learn if not flying - due to use of resources:
    if (of_landing_ctrl.learn_gains) {
      // learn the weights from the file filled with training examples:

      ////////// CDW REMOVED
      //learn_from_file();
      // reset the learn_gains variable to false:
      of_landing_ctrl.learn_gains = false;
    }
    if (of_landing_ctrl.load_weights) {
      // TODO: uncomment:
      // load_weights();
      of_landing_ctrl.load_weights = false;
    }

    // When not flying and in mode module:
    // Reset integrators
    reset_all_vars(); // TODO: uncomment

  } else {

    if (run_control!=1)
    {
      return;
    }

    run_control = 0;

    /***********
    * CONTROL
    ***********/

    // hover laterally:
    guidance_h_set_guided_body_vel(0.0f, 0.0f);

    int32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ; \

    // landing indicates whether the drone is already performing a final landing procedure (flare):
    if (!landing) {

      if (of_landing_ctrl.CONTROL_METHOD == 0) {
        // FIXED GAIN CONTROL, cov_limit for landing:

        // use the divergence for control:
        float err = of_landing_ctrl.divergence_setpoint - divergence;
        of_landing_ctrl.sum_err += err * of_landing_ctrl.igain;
        int32_t thrust = nominal_throttle + of_landing_ctrl.pgain * err * MAX_PPRZ +
                         of_landing_ctrl.sum_err * MAX_PPRZ;
        // make sure the p gain is logged:
        pstate = of_landing_ctrl.pgain;
        pused = pstate;
        // bound thrust:

        // TODO
        //Bound(thrust, 0.8 * nominal_throttle, 1.3 * nominal_throttle);
        Bound(thrust, 0, MAX_PPRZ);

        // histories and cov detection:
        normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
        thrust_history[ind_hist % COV_WINDOW_SIZE] = normalized_thrust;
        divergence_history[ind_hist % COV_WINDOW_SIZE] = divergence;
        int ind_past = (ind_hist % COV_WINDOW_SIZE) - of_landing_ctrl.delay_steps;
        while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
        float past_divergence = divergence_history[ind_past];
        past_divergence_history[ind_hist % COV_WINDOW_SIZE] = past_divergence;
        ind_hist++;
        // determine the covariance for landing detection:
        if (of_landing_ctrl.COV_METHOD == 0) {
          cov_div = get_cov(thrust_history, divergence_history, COV_WINDOW_SIZE);
        } else {
          cov_div = get_cov(past_divergence_history, divergence_history, COV_WINDOW_SIZE);
        }

        if (ind_hist >= COV_WINDOW_SIZE && fabs(cov_div) > of_landing_ctrl.cov_limit) {
          // land by setting 90% nominal thrust:
          //landing = 1;
          thrust = 0.90 * nominal_throttle;
        }
        stabilization_cmd[COMMAND_THRUST] = thrust;
      } else if (of_landing_ctrl.CONTROL_METHOD == 1) {

        // **********************
        // ADAPTIVE GAIN CONTROL:
        // **********************

        // adapt the gains according to the error in covariance:
        float error_cov = of_landing_ctrl.cov_set_point - cov_div;

        // limit the error_cov, which could else become very large:
        if (error_cov > fabs(of_landing_ctrl.cov_set_point)) { error_cov = fabs(of_landing_ctrl.cov_set_point); }

        // if close enough, store inputs:
        if (ind_hist >= COV_WINDOW_SIZE && fabs(error_cov) < of_landing_ctrl.close_to_edge) {
          if (of_landing_ctrl.snapshot) {
            // TODO: watch out, if the texton distribution is the same as the previous one, it will not be saved - and indices of images and the training set will not coincide...
            // video_thread_take_shot(true); // now defined differently...
          }



          ////////// CDW REMOVED
          //save_texton_distribution();
        }

        // adapt the gain:
        pstate -= (of_landing_ctrl.igain_adaptive * pstate) * error_cov;
        if (pstate < MINIMUM_GAIN) { pstate = MINIMUM_GAIN; }

        // regulate the divergence:
        float err = of_landing_ctrl.divergence_setpoint - divergence;
        of_landing_ctrl.sum_err += of_landing_ctrl.igain * err;

        pused = pstate - (of_landing_ctrl.pgain_adaptive * pstate) * error_cov;

        // make sure pused does not become too small, nor grows too fast:
        if (pused < MINIMUM_GAIN) { pused = MINIMUM_GAIN; }
        if (of_landing_ctrl.COV_METHOD == 1 && error_cov > 0.001) {
          pused = 0.5 * pused;
        }

        // set thrust:
        int32_t thrust = nominal_throttle + pused * err * MAX_PPRZ + of_landing_ctrl.sum_err * MAX_PPRZ;

        // histories and cov detection:
        normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
        thrust_history[ind_hist % COV_WINDOW_SIZE] = normalized_thrust;
        divergence_history[ind_hist % COV_WINDOW_SIZE] = divergence;
        int ind_past = (ind_hist % COV_WINDOW_SIZE) - of_landing_ctrl.delay_steps;
        while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
        float past_divergence = divergence_history[ind_past];
        past_divergence_history[ind_hist % COV_WINDOW_SIZE] = 100.0f * past_divergence;
        ind_hist++;

        // only take covariance into account if there are enough samples in the histories:
        if (ind_hist >= COV_WINDOW_SIZE) {
          if (of_landing_ctrl.COV_METHOD == 0) {
            cov_div = get_cov(thrust_history, divergence_history, COV_WINDOW_SIZE);
          } else {
            cov_div = get_cov(past_divergence_history, divergence_history, COV_WINDOW_SIZE);
          }
        } else {
          cov_div = of_landing_ctrl.cov_set_point;
        }

        // TODO: could put a landing condition here based on pstate (if too low)

        // bound thrust:
        Bound(thrust, 0.8 * nominal_throttle, 0.75 * MAX_PPRZ); // was 0.6 0.9
        stabilization_cmd[COMMAND_THRUST] = thrust;
      } else {

        // SSL LANDING: use learned weights for setting the gain on the way down:

        // adapt the p-gain with a low-pass filter to the gain predicted by image appearance:
        // TODO: lp_factor is now the same as used for the divergence. This may not be appropriate
        pstate = predict_gain(texton_distribution_stereoboard);
        of_landing_ctrl.pgain = of_landing_ctrl.lp_factor * of_landing_ctrl.pgain + (1.0f - of_landing_ctrl.lp_factor) *
                                of_landing_ctrl.stable_gain_factor * pstate;
        pused = of_landing_ctrl.pgain;

        // make sure pused does not become too small, nor grows too fast:
        if (of_landing_ctrl.pgain < MINIMUM_GAIN) { of_landing_ctrl.pgain = MINIMUM_GAIN; }

        DEBUG_PRINT("of_landing_ctrl.pgain = %f\n", of_landing_ctrl.pgain);

        // use the divergence for control:
        float err = of_landing_ctrl.divergence_setpoint - divergence;
        int32_t thrust = nominal_throttle + of_landing_ctrl.pgain * err * MAX_PPRZ + of_landing_ctrl.igain *
                         of_landing_ctrl.sum_err * MAX_PPRZ;
        // bound thrust:
        Bound(thrust, 0.8 * nominal_throttle, 0.75 * MAX_PPRZ);

        // histories and cov detection:
        normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
        thrust_history[ind_hist % COV_WINDOW_SIZE] = normalized_thrust;
        divergence_history[ind_hist % COV_WINDOW_SIZE] = divergence;
        int ind_past = (ind_hist % COV_WINDOW_SIZE) - of_landing_ctrl.delay_steps;
        while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
        float past_divergence = divergence_history[ind_past];
        past_divergence_history[ind_hist % COV_WINDOW_SIZE] = past_divergence;
        ind_hist++;
        // determine the covariance for landing detection:
        if (of_landing_ctrl.COV_METHOD == 0) {
          cov_div = get_cov(thrust_history, divergence_history, COV_WINDOW_SIZE);
        } else {
          cov_div = get_cov(past_divergence_history, divergence_history, COV_WINDOW_SIZE);
        }

        // For now no landing procedure. We could do this based on the gain estimate (when it is too low):
        /*
        if (ind_hist >= COV_WINDOW_SIZE && fabs(cov_div) > of_landing_ctrl.cov_limit) {
          // land by setting 90% nominal thrust:
          landing = 1;
          thrust = 0.90 * nominal_throttle;
        }
        */
        stabilization_cmd[COMMAND_THRUST] = thrust;

      }
    } else {
      // FLARE:
      // -----
      // land with 90% nominal thrust:
      int32_t thrust = 0.90 * nominal_throttle;
      Bound(thrust, 0.6 * nominal_throttle, 0.9 * MAX_PPRZ);
      stabilization_cmd[COMMAND_THRUST] = thrust;
    }
  }
}

/**
 * Get the mean value of an array
 * @param[out] mean The mean value
 * @param[in] *a The array
 * @param[in] n Number of elements in the array
 */
float get_mean_array(float *a, int n_elements)
{
  // determine the mean for the vector:
  float mean = 0;
  for (int i = 0; i < n_elements; i++) {
    mean += a[i];
  }
  mean /= n_elements;

  return mean;
}

/**
 * Get the covariance of two arrays
 * @param[out] cov The covariance
 * @param[in] *a The first array
 * @param[in] *b The second array
 * @param[in] n Number of elements in the arrays
 */
float get_cov(float *a, float *b, int n_elements)
{
  // Determine means for each vector:
  float mean_a = get_mean_array(a, n_elements);
  float mean_b = get_mean_array(b, n_elements);

  // Determine the covariance:
  float cov = 0;
  for (int i = 0; i < n_elements; i++) {
    cov += (a[i] - mean_a) * (b[i] - mean_b);
  }

  cov /= n_elements;

  return cov;
}



// Reading from sonar:
static void vertical_ctrl_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  // value from the sonar - normally, this is replaced by the optitrack value in the main loop.
  of_landing_ctrl.agl = distance;
}

// Getting information from optical flow:
static void vertical_ctrl_optical_flow_cb(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp __attribute__((unused)), int16_t flow_x __attribute__((unused)), int16_t flow_y __attribute__((unused)),
    int16_t flow_der_x __attribute__((unused)), int16_t flow_der_y __attribute__((unused)),
    uint8_t quality __attribute__((unused)), float size_divergence, float dist __attribute__((unused)))
{
  divergence_vision = size_divergence;
  has_new_vision_message++;
}
// Getting textons:
static void vertical_ctrl_textons_cb(uint8_t sender_id __attribute__((unused)), uint8_t histogram0, uint8_t histogram1,
                                     uint8_t histogram2, uint8_t histogram3, uint8_t histogram4, uint8_t histogram5,
                                     uint8_t histogram6, uint8_t histogram7, uint8_t histogram8, uint8_t histogram9)
{
  int i;
  texton_distribution_stereoboard[0] = (float) histogram0;
  texton_distribution_stereoboard[1] = (float) histogram1;
  texton_distribution_stereoboard[2] = (float) histogram2;
  texton_distribution_stereoboard[3] = (float) histogram3;
  texton_distribution_stereoboard[4] = (float) histogram4;
  texton_distribution_stereoboard[5] = (float) histogram5;
  texton_distribution_stereoboard[6] = (float) histogram6;
  texton_distribution_stereoboard[7] = (float) histogram7;
  texton_distribution_stereoboard[8] = (float) histogram8;
  texton_distribution_stereoboard[9] = (float) histogram9;

  float sum = 0;
  for (i = 0; i < TEXTONS_N_TEXTONS; i++) {
    sum += texton_distribution_stereoboard[i];
  }

  for (i = 0; i < TEXTONS_N_TEXTONS; i++) {
    texton_distribution_stereoboard[i] /= sum;
  }
}

////////////////////////////////////////////////////////////////////
// Call our controller
void guidance_v_module_init(void)
{
  int i;
  // after re-entering the module, the divergence should be equal to the set point:
  if (ind_hist <= 1) {
    divergence = of_landing_ctrl.divergence_setpoint;
    for (i = 0; i < COV_WINDOW_SIZE; i++) {
      thrust_history[i] = 0;
      divergence_history[i] = 0;
    }
    ind_hist++;
    //dt = 0.0f;
    int32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ;
    stabilization_cmd[COMMAND_THRUST] = nominal_throttle;

  }
}

/**
 * Entering the module (user switched to module)
 */
void guidance_v_module_enter(void)
{
  int i;
  // reset integrator
  of_landing_ctrl.sum_err = 0.0f;
  landing = 0;
  ind_hist = 0;
  previous_err = 0.0f;
  previous_cov_err = 0.0f;
  of_landing_ctrl.agl_lp = 0.0f;
  cov_div = of_landing_ctrl.cov_set_point;
  normalized_thrust = 0.0f;
  divergence = of_landing_ctrl.divergence_setpoint;
  dt = 0.0f;
  for (i = 0; i < COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
  }
}


/**
 * Fit a linear model from samples to target values.
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit
 */
void fit_linear_model(float *targets, float **samples, uint8_t D, uint16_t count, float *params, float *fit_error)
{

  // We will solve systems of the form A x = b,
  // where A = [nx(D+1)] matrix with entries [s1, ..., sD, 1] for each sample (1 is the bias)
  // and b = [nx1] vector with the target values.
  // x in the system are the parameters for the linear regression function.

  // local vars for iterating, random numbers:
  int sam, d;
  uint16_t n_samples = count;
  uint8_t D_1 = D + 1;
  // ensure that n_samples is high enough to ensure a result for a single fit:
  n_samples = (n_samples < D_1) ? D_1 : n_samples;
  // n_samples should not be higher than count:
  n_samples = (n_samples < count) ? n_samples : count;

  // initialize matrices and vectors for the full point set problem:
  // this is used for determining inliers
  float _AA[count][D_1];
  MAKE_MATRIX_PTR(AA, _AA, count);
  float _targets_all[count][1];
  MAKE_MATRIX_PTR(targets_all, _targets_all, count);

  for (sam = 0; sam < count; sam++) {
    for (d = 0; d < D; d++) {
      AA[sam][d] = samples[sam][d];
    }
    if (of_landing_ctrl.use_bias) {
      AA[sam][D] = 1.0f;
    } else {
      AA[sam][D] = 0.0f;
    }
    targets_all[sam][0] = targets[sam];
  }


  // decompose A in u, w, v with singular value decomposition A = u * w * vT.
  // u replaces A as output:
  float _parameters[D_1][1];
  MAKE_MATRIX_PTR(parameters, _parameters, D_1);
  float w[n_samples], _v[D_1][D_1];
  MAKE_MATRIX_PTR(v, _v, D_1);

  pprz_svd_float(AA, w, v, count, D_1);
  pprz_svd_solve_float(parameters, AA, w, v, targets_all, count, D_1, 1);

  // used to determine the error of a set of parameters on the whole set:
  float _bb[count][1];
  MAKE_MATRIX_PTR(bb, _bb, count);
  float _C[count][1];
  MAKE_MATRIX_PTR(C, _C, count);

  // error is determined on the entire set
  // bb = AA * parameters:
  MAT_MUL(count, D_1, 1, bb, AA, parameters);
  // subtract bu_all: C = 0 in case of perfect fit:
  MAT_SUB(count, 1, C, bb, targets_all);
  *fit_error = 0;
  for (sam = 0; sam < count; sam++) {
    *fit_error += abs(C[sam][0]);
  }
  *fit_error /= count;

  for (d = 0; d < D_1; d++) {
    params[d] = parameters[d][0];
  }

}

// General TODO: what happens if n_textons changes?

float predict_gain(float *distribution)
{
  int i;
  float sum;



  sum = 0.0f;
  for (i = 0; i < TEXTONS_N_TEXTONS; i++) {
    sum += weights[i] * distribution[i];
  }
  if (of_landing_ctrl.use_bias) {
    sum += weights[TEXTONS_N_TEXTONS];
  }
  return sum;
}


