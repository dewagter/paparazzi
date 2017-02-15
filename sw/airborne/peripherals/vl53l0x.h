#ifndef VL53L0X_h
#define VL53L0X_h

#include "mcu_periph/i2c.h"

#define ADDRESS_VL53_STD		(0x52)

struct VL53L0X {
  // I2c
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;

  // Private
  uint32_t measurement_timing_budget_us;
  uint8_t stop_variable; // read by init and used when starting measurement

  // Driver
  uint8_t status;
  uint16_t range;
};




// I2C Setup
void setAddress(struct VL53L0X *s, uint8_t new_addr);

// Measurement Setup
typedef enum { VL53L0X_LONG_RANGE, VL53L0X_HIGH_SPEED, VL53L0X_HIGH_ACCURACY, VL53L0X_DEFAULT } VL53L0X_SETUP;
void init(struct VL53L0X *s, struct i2c_periph *i2c_p, VL53L0X_SETUP setup);


// Change Quality
void setSignalRateLimit(struct VL53L0X *s, float limit_Mcps);
typedef enum { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;
void setVcselPulsePeriod(struct VL53L0X *s, vcselPeriodType type, uint8_t period_pclks);
void setMeasurementTimingBudget(struct VL53L0X *s, uint32_t budget_us);



void startContinuous(struct VL53L0X *s, uint32_t period_ms);
//void stopContinuous(struct VL53L0X *s);
uint16_t readRangeContinuousMillimeters(struct VL53L0X *s);
uint16_t readRangeSingleMillimeters(struct VL53L0X *s);





#endif



