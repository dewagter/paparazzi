#include "modules/sensors/ground_detection_switch.h"

#include "subsystems/abi.h"

#define SENDER_ID 1

uint8_t counter;
uint16_t gpio_val;
bool ground_detect;

#ifndef GPIO_PORT
#define GPIO_PORT GPIOC
#endif

#ifndef GPIO_CHANNEL
#define GPIO_CHANNEL GPIO12
#endif

#ifndef GPIO_LOW
#define GPIO_LOW 0
#endif

#ifndef GPIO_HIHG
#define GPIO_HIGH 1
#endif

#ifndef COUNTER_THRESHOLD
#define COUNTER_THRESHOLD 3
#endif

void ground_detection_switch_init( void ) {
  counter = 0;
  gpio_val = GPIO_HIGH;
  ground_detect = false;

  gpio_setup_input_pullup(GPIO_PORT, GPIO_CHANNEL);
}

void ground_detection_switch_periodic( void ) {
  gpio_val = gpio_get(GPIO_PORT, GPIO_CHANNEL);
  if (gpio_val == GPIO_LOW) {
    counter++;
    if (counter >= COUNTER_THRESHOLD) {
      ground_detect = TRUE;
    }
  } else {
    counter = 0;
    ground_detect = FALSE;
  }
  AbiSendMsgGROUND_DETECT(SENDER_ID, &ground_detect);
}


