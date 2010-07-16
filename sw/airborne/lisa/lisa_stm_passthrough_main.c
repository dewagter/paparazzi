/*
 * $Id$
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "init_hw.h"
#include "sys_time.h"
#include "downlink.h"
#include "booz/booz2_commands.h"
#include "booz/booz_actuators.h"
#include "booz/booz_imu.h"
#include "booz_radio_control.h"
#include "actuators/booz_actuators_pwm.h"
#include "lisa/lisa_overo_link.h"

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

static inline void on_gyro_accel_event(void);
static inline void on_mag_event(void);

static inline void main_on_overo_msg_received(void);
static inline void main_on_overo_link_lost(void);

static bool_t new_radio_msg;

int main(void) {

	main_init();

	while (1) {
		if (sys_time_periodic())
			main_periodic();
		main_event();
	}

	return 0;
}

static void on_rc_message(void) {
  new_radio_msg = TRUE;
}

static inline void main_init(void) {

	hw_init();
	sys_time_init();
	booz_imu_init();
	radio_control_init();
	booz_actuators_pwm_hw_init();
	overo_link_init();
}

static inline void main_periodic(void) {

	booz_imu_periodic();
	OveroLinkPeriodic(main_on_overo_link_lost);
	RunOnceEvery(10, {LED_PERIODIC(); DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);radio_control_periodic();});
}

static inline void main_event(void) {

	BoozImuEvent(on_gyro_accel_event, on_mag_event);
	OveroLinkEvent(main_on_overo_msg_received);
	RadioControlEvent(on_rc_message);
}

static inline void main_on_overo_msg_received(void) {

	if (new_radio_msg) overo_link.up.msg.valid.rc = 1;
	else overo_link.up.msg.valid.rc = 0;
        new_radio_msg = FALSE;

	overo_link.up.msg.valid.imu = 1;

	RATES_COPY(overo_link.up.msg.gyro, booz_imu.gyro);

	VECT3_COPY(overo_link.up.msg.accel, booz_imu.accel);

	VECT3_COPY(overo_link.up.msg.mag, booz_imu.mag);

	overo_link.up.msg.rc_pitch = radio_control.values[RADIO_CONTROL_PITCH];
	overo_link.up.msg.rc_roll = radio_control.values[RADIO_CONTROL_ROLL];
	overo_link.up.msg.rc_yaw = radio_control.values[RADIO_CONTROL_YAW];
	overo_link.up.msg.rc_thrust = radio_control.values[RADIO_CONTROL_THROTTLE];
	overo_link.up.msg.rc_mode = radio_control.values[RADIO_CONTROL_MODE];
	overo_link.up.msg.rc_kill = radio_control.values[RADIO_CONTROL_KILL];
	overo_link.up.msg.rc_gear = radio_control.values[RADIO_CONTROL_GEAR];
	overo_link.up.msg.rc_aux3 = radio_control.values[RADIO_CONTROL_AUX3];
	overo_link.up.msg.rc_aux4 = radio_control.values[RADIO_CONTROL_AUX4];
	overo_link.up.msg.rc_status = radio_control.status;

	for (int i = 0; i < LISA_PWM_OUTPUT_NB; i++)
	  booz_actuators_pwm_values[i] = overo_link.down.msg.pwm_outputs_usecs[i];
	booz_actuators_pwm_commit();
}

static inline void main_on_overo_link_lost(void) {

}

static inline void on_gyro_accel_event(void) {
	static uint8_t cnt;

	BoozImuScaleGyro();
	BoozImuScaleAccel();

	LED_TOGGLE(2);
	cnt++;
	if (cnt > 15) cnt = 0;

	if (cnt == 0) {
		DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel,
					&booz_imu.gyro_unscaled.p,
					&booz_imu.gyro_unscaled.q,
					&booz_imu.gyro_unscaled.r);

		DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel,
					&booz_imu.accel_unscaled.x,
					&booz_imu.accel_unscaled.y,
					&booz_imu.accel_unscaled.z);
	} else if (cnt == 7) {
		DOWNLINK_SEND_BOOZ2_GYRO(DefaultChannel,
					&booz_imu.gyro.p,
					&booz_imu.gyro.q,
					&booz_imu.gyro.r);

		DOWNLINK_SEND_BOOZ2_ACCEL(DefaultChannel,
					&booz_imu.accel.x,
					&booz_imu.accel.y,
					&booz_imu.accel.z);
	}
}

static inline void on_mag_event(void) {
	static uint8_t cnt;

	BoozImuScaleMag();

	cnt++;
	if (cnt % 2) {
		DOWNLINK_SEND_BOOZ2_MAG(DefaultChannel,
					&booz_imu.mag.x,
					&booz_imu.mag.y,
					&booz_imu.mag.z);
	} else {
		DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel,
					&booz_imu.mag_unscaled.x,
					&booz_imu.mag_unscaled.y,
					&booz_imu.mag_unscaled.z);
	}
}

