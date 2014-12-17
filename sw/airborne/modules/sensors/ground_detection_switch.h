#ifndef GROUND_DETECTION_SWITCH_H
#define GROUND_DETECTION_SWITCH_H

#include "messages.h"
#include "subsystems/datalink/telemetry.h"

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio_arch.h"
#include BOARD_CONFIG

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

extern uint8_t counter;
extern bool ground_detect;

void ground_detection_switch_init( void );
void ground_detection_switch_periodic( void );

#endif /* GROUND_DETECTION_SWITCH_H */
