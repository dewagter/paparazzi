// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include "peripherals/vl53l0x.h"



// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)


// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
static inline uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static inline uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static inline uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static inline uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}





static inline uint32_t getMeasurementTimingBudget(struct VL53L0X *s);

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection

struct SequenceStepEnables
{
  bool tcc, msrc, dss, pre_range, final_range;
};

struct SequenceStepTimeouts
{
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
};


static void getSpadInfo(struct VL53L0X *s, uint8_t * count, bool * type_is_aperture);
static void getSequenceStepEnables(struct VL53L0X *s, struct SequenceStepEnables * enables);
static void getSequenceStepTimeouts(struct VL53L0X *s, struct SequenceStepEnables const * enables, struct SequenceStepTimeouts * timeouts);
static void performSingleRefCalibration(struct VL53L0X *s, uint8_t vhv_init_byte);
static uint8_t getVcselPulsePeriod(struct VL53L0X *s, vcselPeriodType type);


enum regAddr
{
  SYSRANGE_START                              = 0x00,

  SYSTEM_THRESH_HIGH                          = 0x0C,
  SYSTEM_THRESH_LOW                           = 0x0E,

  SYSTEM_SEQUENCE_CONFIG                      = 0x01,
  SYSTEM_RANGE_CONFIG                         = 0x09,
  SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

  SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

  GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

  SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

  RESULT_INTERRUPT_STATUS                     = 0x13,
  RESULT_RANGE_STATUS                         = 0x14,

  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
  RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
  RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

  ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

  I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

  MSRC_CONFIG_CONTROL                         = 0x60,

  PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
  PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
  PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

  FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
  FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

  PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
  PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

  PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

  SYSTEM_HISTOGRAM_BIN                        = 0x81,
  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
  HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

  FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

  MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

  SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
  IDENTIFICATION_MODEL_ID                     = 0xC0,
  IDENTIFICATION_REVISION_ID                  = 0xC2,

  OSC_CALIBRATE_VAL                           = 0xF8,

  GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

  GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
  DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
  POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

  ALGO_PHASECAL_LIM                           = 0x30,
  ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

static inline void waitReady(struct i2c_transaction *t) {
  while(t->status < I2CTransSuccess)
  {
  }
}


static inline void writeReg(struct VL53L0X *s, uint8_t reg, uint8_t data) {
  s->i2c_trans.buf[0] = reg;
  s->i2c_trans.buf[1] = data;
  i2c_transmit(s->i2c_p, &s->i2c_trans, s->i2c_trans.slave_addr, 2);
  waitReady(&s->i2c_trans);
}

static inline void writeReg16Bit(struct VL53L0X *s, uint8_t reg, uint16_t data) {
  s->i2c_trans.buf[0] = reg;
  s->i2c_trans.buf[1] = (data >> 8) & 0xFF;
  s->i2c_trans.buf[2] = (data & 0xFF);
  i2c_transmit(s->i2c_p, &s->i2c_trans, s->i2c_trans.slave_addr, 2);
  waitReady(&s->i2c_trans);
}

static inline uint8_t readReg(struct VL53L0X *s, uint8_t reg) {
  s->i2c_trans.buf[0] = reg;
  i2c_transceive(s->i2c_p, &s->i2c_trans, s->i2c_trans.slave_addr, 1, 1);
  // wait ready
  waitReady(&s->i2c_trans);
  return s->i2c_trans.buf[0];
}

static inline uint16_t readReg16Bit(struct VL53L0X *s, uint8_t reg) {
  uint16_t ret = 0;
  s->i2c_trans.buf[0] = reg;
  i2c_transceive(s->i2c_p, &s->i2c_trans, s->i2c_trans.slave_addr, 1, 2);
  // wait ready
  waitReady(&s->i2c_trans);
  ret = ((uint16_t)s->i2c_trans.buf[0]) << 8; // value high byte
  ret += s->i2c_trans.buf[1];
  return ret;
}

// Public Methods //////////////////////////////////////////////////////////////

void setAddress(struct VL53L0X *s, uint8_t new_addr) {
  writeReg(s, I2C_SLAVE_DEVICE_ADDRESS, (new_addr>>1) & 0x7F);
  s->i2c_trans.slave_addr = new_addr;
}



// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
void init(struct VL53L0X *s, struct i2c_periph *i2c_p, VL53L0X_SETUP setup) {

  s->i2c_p = i2c_p;
  s->status = 0;
  s->i2c_trans.status = I2CIdle;

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  writeReg(s, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      readReg(s, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0

  // "Set I2C standard mode"
  writeReg(s, 0x88, 0x00);

  writeReg(s, 0x80, 0x01);
  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x00, 0x00);
  s->stop_variable = readReg(s, 0x91);
  writeReg(s, 0x00, 0x01);
  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  writeReg(s, MSRC_CONFIG_CONTROL, readReg(s, MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(s, 0.25);

  writeReg(s, SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  getSpadInfo(s, &spad_count, &spad_type_is_aperture);

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];


  // readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  s->i2c_trans.buf[0] = GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
  i2c_transceive(s->i2c_p, &s->i2c_trans, s->i2c_trans.slave_addr, 1, 6);
  waitReady(&s->i2c_trans);

  for (int i=0; i<6; i++)
    ref_spad_map[i] = s->i2c_trans.buf[i+1];


  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  writeReg(s, 0xFF, 0x01);
  writeReg(s, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(s, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(s, 0xFF, 0x00);
  writeReg(s, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }


  s->i2c_trans.buf[0] = GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
  for (int i=0; i<6; i++)
    s->i2c_trans.buf[i+1] = ref_spad_map[i];
  i2c_transmit(s->i2c_p, &s->i2c_trans, s->i2c_trans.slave_addr, 1 + 6);
  waitReady(&s->i2c_trans);


  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x00, 0x00);

  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x09, 0x00);
  writeReg(s, 0x10, 0x00);
  writeReg(s, 0x11, 0x00);

  writeReg(s, 0x24, 0x01);
  writeReg(s, 0x25, 0xFF);
  writeReg(s, 0x75, 0x00);

  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x4E, 0x2C);
  writeReg(s, 0x48, 0x00);
  writeReg(s, 0x30, 0x20);

  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x30, 0x09);
  writeReg(s, 0x54, 0x00);
  writeReg(s, 0x31, 0x04);
  writeReg(s, 0x32, 0x03);
  writeReg(s, 0x40, 0x83);
  writeReg(s, 0x46, 0x25);
  writeReg(s, 0x60, 0x00);
  writeReg(s, 0x27, 0x00);
  writeReg(s, 0x50, 0x06);
  writeReg(s, 0x51, 0x00);
  writeReg(s, 0x52, 0x96);
  writeReg(s, 0x56, 0x08);
  writeReg(s, 0x57, 0x30);
  writeReg(s, 0x61, 0x00);
  writeReg(s, 0x62, 0x00);
  writeReg(s, 0x64, 0x00);
  writeReg(s, 0x65, 0x00);
  writeReg(s, 0x66, 0xA0);

  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x22, 0x32);
  writeReg(s, 0x47, 0x14);
  writeReg(s, 0x49, 0xFF);
  writeReg(s, 0x4A, 0x00);

  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x7A, 0x0A);
  writeReg(s, 0x7B, 0x00);
  writeReg(s, 0x78, 0x21);

  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x23, 0x34);
  writeReg(s, 0x42, 0x00);
  writeReg(s, 0x44, 0xFF);
  writeReg(s, 0x45, 0x26);
  writeReg(s, 0x46, 0x05);
  writeReg(s, 0x40, 0x40);
  writeReg(s, 0x0E, 0x06);
  writeReg(s, 0x20, 0x1A);
  writeReg(s, 0x43, 0x40);

  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x34, 0x03);
  writeReg(s, 0x35, 0x44);

  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x31, 0x04);
  writeReg(s, 0x4B, 0x09);
  writeReg(s, 0x4C, 0x05);
  writeReg(s, 0x4D, 0x04);

  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x44, 0x00);
  writeReg(s, 0x45, 0x20);
  writeReg(s, 0x47, 0x08);
  writeReg(s, 0x48, 0x28);
  writeReg(s, 0x67, 0x00);
  writeReg(s, 0x70, 0x04);
  writeReg(s, 0x71, 0x01);
  writeReg(s, 0x72, 0xFE);
  writeReg(s, 0x76, 0x00);
  writeReg(s, 0x77, 0x00);

  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x0D, 0x01);

  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x80, 0x01);
  writeReg(s, 0x01, 0xF8);

  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x8E, 0x01);
  writeReg(s, 0x00, 0x01);
  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  writeReg(s, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(s, GPIO_HV_MUX_ACTIVE_HIGH, readReg(s, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  writeReg(s, SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  s->measurement_timing_budget_us = getMeasurementTimingBudget(s);

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  writeReg(s, SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(s, s->measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  writeReg(s, SYSTEM_SEQUENCE_CONFIG, 0x01);
  performSingleRefCalibration(s, 0x40);

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  writeReg(s, SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(s, 0x00);

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  writeReg(s, SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end



  if (setup ==  VL53L0X_LONG_RANGE)
  {
    // lower the return signal rate limit (default is 0.25 MCPS)
    setSignalRateLimit(s, 0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    setVcselPulsePeriod(s, VcselPeriodPreRange, 18);
    setVcselPulsePeriod(s, VcselPeriodFinalRange, 14);
  }
    
  if (setup ==  VL53L0X_HIGH_SPEED)
  {
    // reduce timing budget to 20 ms (default is about 33 ms)
    setMeasurementTimingBudget(s, 20000);
  }
  
  if (setup ==  VL53L0X_HIGH_ACCURACY)
  {
    // increase timing budget to 200 ms
    setMeasurementTimingBudget(s, 200000);
  }

  // setMeasurementTimingBudget(s, 50000);

  return;
}



// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
void setSignalRateLimit(struct VL53L0X *s, float limit_Mcps)
{
  if (limit_Mcps < 0)
	limit_Mcps = 0;
  if (limit_Mcps > 511.99)
	limit_Mcps = 511.99;

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  writeReg16Bit(s, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
}


// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
void setMeasurementTimingBudget(struct VL53L0X *s, uint32_t budget_us)
{
  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget)
    budget_us = MinTimingBudget;

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(s, &enables);
  getSequenceStepTimeouts(s, &enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks( final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(s, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    s->measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return;
}


// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t getMeasurementTimingBudget(struct VL53L0X *s)
{
  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(s, &enables);
  getSequenceStepTimeouts(s, &enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  s->measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
void setVcselPulsePeriod(struct VL53L0X *s, vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  getSequenceStepEnables(s, &enables);
  getSequenceStepTimeouts(s, &enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        writeReg(s, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        writeReg(s, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        writeReg(s, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        writeReg(s, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return;
    }
    writeReg(s, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    writeReg(s, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    writeReg16Bit(s, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    writeReg(s, MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(s, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(s, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(s, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(s, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(s, 0xFF, 0x01);
        writeReg(s, ALGO_PHASECAL_LIM, 0x30);
        writeReg(s, 0xFF, 0x00);
        break;

      case 10:
        writeReg(s, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(s, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(s, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(s, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(s, 0xFF, 0x01);
        writeReg(s, ALGO_PHASECAL_LIM, 0x20);
        writeReg(s, 0xFF, 0x00);
        break;

      case 12:
        writeReg(s, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(s, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(s, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(s, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(s, 0xFF, 0x01);
        writeReg(s, ALGO_PHASECAL_LIM, 0x20);
        writeReg(s, 0xFF, 0x00);
        break;

      case 14:
        writeReg(s, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(s, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(s, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(s, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(s, 0xFF, 0x01);
        writeReg(s, ALGO_PHASECAL_LIM, 0x20);
        writeReg(s, 0xFF, 0x00);
        break;

      default:
        // invalid period
        return;
    }

    // apply new VCSEL period
    writeReg(s, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(s, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(s, s->measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = readReg(s, SYSTEM_SEQUENCE_CONFIG);
  writeReg(s, SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(s, 0x0);
  writeReg(s, SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t getVcselPulsePeriod(struct VL53L0X *s, vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(readReg(s, PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(readReg(s, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void startContinuous(struct VL53L0X *s, uint32_t period_ms)
{
  writeReg(s, 0x80, 0x01);
  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x00, 0x00);
  writeReg(s, 0x91, s->stop_variable);
  writeReg(s, 0x00, 0x01);
  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = readReg16Bit(s, OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    s->i2c_trans.buf[0] = SYSTEM_INTERMEASUREMENT_PERIOD;
    s->i2c_trans.buf[1] = ((period_ms >> 24) & 0xFF); // value highest byte
    s->i2c_trans.buf[2] = ((period_ms >> 16) & 0xFF);
    s->i2c_trans.buf[3] = ((period_ms >>  8) & 0xFF);
    s->i2c_trans.buf[4] = ( period_ms        & 0xFF); // value lowest byte
	i2c_transmit(s->i2c_p, &s->i2c_trans, s->i2c_trans.slave_addr, 5);
    waitReady(&s->i2c_trans);


    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    writeReg(s, SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    writeReg(s, SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

/*
// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void stopContinuous(struct VL53L0X *s)
{
  writeReg(s, SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x00, 0x00);
  writeReg(s, 0x91, 0x00);
  writeReg(s, 0x00, 0x01);
  writeReg(s, 0xFF, 0x00);
}
*/

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)

uint16_t readRangeContinuousMillimeters(struct VL53L0X *s)
{

  while ((readReg(s, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
// TODO
//    if (checkTimeoutExpired())
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = readReg16Bit(s, RESULT_RANGE_STATUS + 10);

  writeReg(s, SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t readRangeSingleMillimeters(struct VL53L0X *s)
{
  writeReg(s, 0x80, 0x01);
  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x00, 0x00);
  writeReg(s, 0x91, s->stop_variable);
  writeReg(s, 0x00, 0x01);
  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x80, 0x00);

  writeReg(s, SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"

  while (readReg(s, SYSRANGE_START) & 0x01)
  {
// TODO
//    if (checkTimeoutExpired())
  }

  return readRangeContinuousMillimeters(s);
}


// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
void getSpadInfo(struct VL53L0X *s, uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  writeReg(s, 0x80, 0x01);
  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x00, 0x00);

  writeReg(s, 0xFF, 0x06);
  writeReg(s, 0x83, readReg(s, 0x83) | 0x04);
  writeReg(s, 0xFF, 0x07);
  writeReg(s, 0x81, 0x01);

  writeReg(s, 0x80, 0x01);

  writeReg(s, 0x94, 0x6b);
  writeReg(s, 0x83, 0x00);

//  startTimeout();
  while (readReg(s, 0x83) == 0x00)
  {
// TODO
//    if (checkTimeoutExpired()) { return false; }
  }
  writeReg(s, 0x83, 0x01);
  tmp = readReg(s, 0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(s, 0x81, 0x00);
  writeReg(s, 0xFF, 0x06);
  writeReg(s, 0x83, readReg(s,  0x83  & ~0x04));
  writeReg(s, 0xFF, 0x01);
  writeReg(s, 0x00, 0x01);

  writeReg(s, 0xFF, 0x00);
  writeReg(s, 0x80, 0x00);

  return;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void getSequenceStepEnables(struct VL53L0X *s, struct SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(s, SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void getSequenceStepTimeouts(struct VL53L0X *s, struct SequenceStepEnables const * enables, struct SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(s, VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = readReg(s, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16Bit(s, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(s, VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16Bit(s, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}


// based on VL53L0X_perform_single_ref_calibration()
void performSingleRefCalibration(struct VL53L0X *s, uint8_t vhv_init_byte)
{
  writeReg(s, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

//TODO
//  startTimeout();
  while ((readReg(s, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
//TODO
//    if (checkTimeoutExpired()) { return false; }
  }

  writeReg(s, SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeReg(s, SYSRANGE_START, 0x00);

  return;
}

