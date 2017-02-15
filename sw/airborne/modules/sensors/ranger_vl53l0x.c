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
 * @file "modules/sensors/ranger_vl53l0x.c"
 * @author C. De Wagter
 * Sensor driver for VL53L0X laser range sensor
 */

#include "modules/sensors/ranger_vl53l0x.h"
#include "peripherals/vl53l0x.h"


#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


uint16_t vl53l0x_range;

static struct VL53L0X vl53l0x_data;

#ifndef VL53L0X_I2C_DEV
#define VL53L0X_I2C_DEV i2c0
#endif



void ranger_vl53_init(void) {
  vl53l0x_range = 0xffff;

  vl53l0x_data.i2c_p = &VL53L0X_I2C_DEV;
  vl53l0x_data.i2c_trans.slave_addr = ADDRESS_VL53_STD;

#ifdef ADDRESS_VL53_D1
  setAddress(&s1, ADDRESS_VL53_D1);
#endif

  // Setup Laser
  init(&vl53l0x_data, &VL53L0X_I2C_DEV, VL53L0X_LONG_RANGE);

  // Start
  startContinuous(&vl53l0x_data, 0); // Period 0 = back-to-back mode

  // Mark Initialized
  vl53l0x_data.status = 1;
}

void ranger_vl53_periodic(void) {
#if 1
  uint16_t range = vl53l0x_range;
  DOWNLINK_SEND_CAMERA_SNAPSHOT(DefaultChannel, DefaultDevice,
                              &range);
#endif
#if 0
  <message name="RANGEFINDER" id="29">
    <field name="range" type="uint16" unit="cm"/>
    <field name="z_dot" type="float" unit="m/s"/>
    <field name="z_dot_sum_err" type="float" unit="m/s"/>
    <field name="z_dot_setpoint" type="float" unit="m/s"/>
    <field name="z_sum_err" type="float" unit="m/s"/>
    <field name="z_setpoint" type="float" unit="m"/>
    <field name="flying" type="uint8" unit="bool"/>
  </message>

  <message name="TMP_STATUS" id="86">
    <field name="itemp"  type="uint16"/>
    <field name="temp"   type="float" unit="deg_celsius" format="%.2f"/>
  </message>

  <message name="CAMERA_SNAPSHOT" id="128">
    <field name="snapshot_image_number" type="uint16"/>
  </message>

  <message name="LIDAR" id="234">
    <field name="distance" type="float" unit="m"/>
    <field name="status" type="uint8" values="REQ|READ|PARSE"/>
    <field name="trans_status" type="uint8" values="Pending|Running|Success|Failed|Done"/>
  </message>

 <message name="SONAR" id="236">
    <field name="sonar_meas" type="uint16"/>
    <field name="sonar_distance" type="float" unit="m"/>
  </message>


#endif
}

void ranger_vl53_event(void) {
  // If Initialized
  if (vl53l0x_data.status == 0)
    return;

  // If I2C Ready
  if (vl53l0x_data.i2c_trans.status >= I2CTransSuccess) {
    switch (vl53l0x_data.status) {
      default: // Startup
        // Check Laser Status
        s->i2c_trans.buf[0] = RESULT_INTERRUPT_STATUS;
        i2c_transceive(vl53l0x_data.i2c_p, &vl53l0x_data.i2c_trans, vl53l0x_data.i2c_trans.slave_addr, 1, 1);
        vl53l0x_data.status = 2;
        break;
      case 2: // (ret  & 0x07) == 0)
        // 
        uint8_t ret = vl53l0x_data.i2c_trans.buf[0];
        if ((ret & 0x07) == 0) { // If not ready yet with measurement
          // Check Laser Status (again)
          ret = readReg(s, RESULT_INTERRUPT_STATUS);
        } else { // If ready; ask the actual result
          // assumptions: Linearity Corrective Gain is 1000 (default);
          // fractional ranging is not enabled
          uint16_t range = readReg16Bit(s, RESULT_RANGE_STATUS + 10);
          vl53l0x_data.status = 3;
        }
        break;
      case 3: // Result obtained: 
        // Fetch result:
        uint16_t range = 0;
        range = ((uint16_t)vl53l0x_data.i2c_trans.buf[0]) << 8; // value high byte
        ret  += ((uint16_t)vl53l0x_data.i2c_trans.buf[1]);
        vl53l0x_data

        // Clear interrupt to start next
        vl53l0x_data.i2c_trans.buf[0] = SYSTEM_INTERRUPT_CLEAR;
        vl53l0x_data.i2c_trans.buf[1] = 0x01;
        i2c_transmit(vl53l0x_data.i2c_p, &vl53l0x_data.i2c_trans, vl53l0x_data.i2c_trans.slave_addr, 2);
        vl53l0x_data.status = 1;
        break;
  }

/*



  return range;
*/
}


