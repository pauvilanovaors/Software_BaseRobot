   /*
 * tof.c
 *
 *  Created on: Aug 15, 2022
 *      Author: pauvi
 */

#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include "stm32f4xx_hal.h"
#include "tof.h"

static unsigned char stop_variable;
static uint32_t measurement_timing_budget_us;

static uint8_t readReg(uint8_t regAddr, uint8_t devAddr);
static uint16_t readReg16(uint8_t regAddr, uint8_t devAddr);
static void readMulti(uint8_t regAddr, uint8_t *pBuf, int size, uint8_t devAddr);
static void writeReg16(uint8_t regAddr, uint16_t data, uint8_t devAddr);
static void writeReg(uint8_t regAddr, uint8_t data, uint8_t devAddr);
static void writeRegList(uint8_t *dataList, uint8_t devAddr);
static int initSensor(int bLongRangeMode, uint8_t devAddr);
static int performSingleRefCalibration(uint8_t vhv_init_byte,uint8_t devAddr);
static int setMeasurementTimingBudget(uint32_t budget_us, uint8_t devAddr);

#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

#define SEQUENCE_ENABLE_FINAL_RANGE 0x80
#define SEQUENCE_ENABLE_PRE_RANGE   0x40
#define SEQUENCE_ENABLE_TCC         0x10
#define SEQUENCE_ENABLE_DSS         0x08
#define SEQUENCE_ENABLE_MSRC        0x04

typedef enum vcselperiodtype { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;
static int setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks,uint8_t devAddr);
extern I2C_HandleTypeDef hi2c1;
typedef struct tagSequenceStepTimeouts
    {
      uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

      uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
      uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    } SequenceStepTimeouts;

// VL53L0X internal registers
#define REG_IDENTIFICATION_MODEL_ID		0xc0
#define REG_IDENTIFICATION_REVISION_ID		0xc2
#define REG_SYSRANGE_START			0x00

#define REG_RESULT_INTERRUPT_STATUS 		0x13
#define RESULT_RANGE_STATUS      		0x14
#define ALGO_PHASECAL_LIM                       0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT            0x30

#define GLOBAL_CONFIG_VCSEL_WIDTH               0x32
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48

#define PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x57

#define REG_MSRC_CONFIG_CONTROL                 0x60
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44
#define SYSRANGE_START                          0x00
#define SYSTEM_SEQUENCE_CONFIG                  0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define RESULT_INTERRUPT_STATUS                 0x13
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define SYSTEM_INTERRUPT_CLEAR                  0x0B
#define I2C_SLAVE_DEVICE_ADDRESS                0x8A
//
// Opens a file system handle to the I2C device
// reads the calibration data and sets the device
// into auto sensing mode
//
int tofInit( int bLongRange, uint8_t devAddr)
{
	return initSensor(bLongRange,devAddr);
} /* tofInit() */

//
// Read a pair of registers as a 16-bit value
//
static uint16_t readReg16(uint8_t regAddr, uint8_t devAddr)
{
	uint8_t datatmp[2];
	uint16_t data; //uint16_t == unsigned short

	//1. Write register address in VL53L0X
	HAL_I2C_Master_Transmit(&hi2c1,devAddr,&regAddr,1,1000);
	//2. Read data from 2 byte register
	HAL_I2C_Master_Receive(&hi2c1,devAddr+1,datatmp,2,1000);

	data = (datatmp[0]<<8) + datatmp[1];

	return data;

} /* readReg16() */

//
// Read a single register value from I2C device
//
static uint8_t readReg(uint8_t regAddr, uint8_t devAddr)
{
	uint8_t data;  //unsigned char == uint8_t

	//1. Write register address in VL53L0X
	HAL_I2C_Master_Transmit(&hi2c1,devAddr,&regAddr,1,1000);
	//2. Read data from 1 byte register
	HAL_I2C_Master_Receive(&hi2c1,devAddr+1,&data,1,1000);

	return data;
} /* ReadReg() */

static void readMulti(uint8_t regAddr, uint8_t *pBuf, int size, uint8_t devAddr)
{
    //1. Write index of register
	HAL_I2C_Master_Transmit(&hi2c1,devAddr,&regAddr,1,1000);

	//2. Read size bytes
	HAL_I2C_Master_Receive(&hi2c1,devAddr+1,pBuf,size,1000);

} /* readMulti() */

static void writeMulti(uint8_t regAddr, uint8_t *pBuf, int size, uint8_t devAddr)
{
	uint8_t buf[size + 1];
	buf[0] = regAddr;
	for(int i = 1; i<size+1;i++){
		buf[i] = *(pBuf++);
	}
	HAL_I2C_Master_Transmit(&hi2c1,devAddr,buf,size+1,1000);

} /* writeMulti() */
//
// Write a 16-bit value to a register
//
void writeReg16(uint8_t regAddr, uint16_t data, uint8_t devAddr)
{
	uint8_t datatmp[4];

	datatmp[0] = regAddr;
	datatmp[1] = (uint8_t)(data >> 8); // MSB first
	datatmp[2] = (uint8_t)data;
	HAL_I2C_Master_Transmit(&hi2c1,devAddr,datatmp,3,1000);

} /* writeReg16() */
//
// Write a single register/value pair
//
static void writeReg(uint8_t regAddr, uint8_t data, uint8_t devAddr)
{
	uint8_t datatmp[2];

	datatmp[0] = regAddr;
	datatmp[1] = data;

	HAL_I2C_Master_Transmit(&hi2c1,devAddr,datatmp,2,1000);
} /* writeReg() */

//
// Write a list of register/value pairs to the I2C device
//


void writeRegList(uint8_t *dataList, uint8_t devAddr)
{
	uint8_t pcount = *dataList++;

	while(pcount){
		HAL_I2C_Master_Transmit(&hi2c1,devAddr,dataList,2,1000);
		dataList += 2;
		pcount--;
	}
} /* writeRegList() */

//
// Register init lists consist of the count followed by register/value pairs
//
unsigned char ucI2CMode[] = {4, 0x88,0x00, 0x80,0x01, 0xff,0x01, 0x00,0x00};
unsigned char ucI2CMode2[] = {3, 0x00,0x01, 0xff,0x00, 0x80,0x00};
unsigned char ucSPAD0[] = {4, 0x80,0x01, 0xff,0x01, 0x00,0x00, 0xff,0x06};
unsigned char ucSPAD1[] = {5, 0xff,0x07, 0x81,0x01, 0x80,0x01, 0x94,0x6b, 0x83,0x00};
unsigned char ucSPAD2[] = {4, 0xff,0x01, 0x00,0x01, 0xff,0x00, 0x80,0x00};
unsigned char ucSPAD[] = {5, 0xff,0x01, 0x4f,0x00, 0x4e,0x2c, 0xff,0x00, 0xb6,0xb4};
unsigned char ucDefTuning[] = {80, 0xff,0x01, 0x00,0x00, 0xff,0x00, 0x09,0x00,
0x10,0x00, 0x11,0x00, 0x24,0x01, 0x25,0xff, 0x75,0x00, 0xff,0x01, 0x4e,0x2c,
0x48,0x00, 0x30,0x20, 0xff,0x00, 0x30,0x09, 0x54,0x00, 0x31,0x04, 0x32,0x03,
0x40,0x83, 0x46,0x25, 0x60,0x00, 0x27,0x00, 0x50,0x06, 0x51,0x00, 0x52,0x96,
0x56,0x08, 0x57,0x30, 0x61,0x00, 0x62,0x00, 0x64,0x00, 0x65,0x00, 0x66,0xa0,
0xff,0x01, 0x22,0x32, 0x47,0x14, 0x49,0xff, 0x4a,0x00, 0xff,0x00, 0x7a,0x0a,
0x7b,0x00, 0x78,0x21, 0xff,0x01, 0x23,0x34, 0x42,0x00, 0x44,0xff, 0x45,0x26,
0x46,0x05, 0x40,0x40, 0x0e,0x06, 0x20,0x1a, 0x43,0x40, 0xff,0x00, 0x34,0x03,
0x35,0x44, 0xff,0x01, 0x31,0x04, 0x4b,0x09, 0x4c,0x05, 0x4d,0x04, 0xff,0x00,
0x44,0x00, 0x45,0x20, 0x47,0x08, 0x48,0x28, 0x67,0x00, 0x70,0x04, 0x71,0x01,
0x72,0xfe, 0x76,0x00, 0x77,0x00, 0xff,0x01, 0x0d,0x01, 0xff,0x00, 0x80,0x01,
0x01,0xf8, 0xff,0x01, 0x8e,0x01, 0x00,0x01, 0xff,0x00, 0x80,0x00};

void SetDevAddr(uint8_t new_addr, uint8_t last_addr){

	/*VL53L0X_WrByte(Dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS,
			DeviceAddress / 2);*/
	writeReg(I2C_SLAVE_DEVICE_ADDRESS,new_addr / 2,last_addr);
	/*
	 *  1. Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
		2. Keep sensor #1 awake by keeping XSHUT pin high
		3. Put all other sensors into shutdown by pulling XSHUT pins low
		4. Initialize sensor #1 with SetDevAddr. Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
		5. Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
		6 .Initialize sensor #2 with lox.begin(new_devAddress) Pick any number but 0x29 and whatever you set the first sensor to
		7. Repeat for each sensor, turning each one on, setting a unique address
	 */
}


static int getSpadInfo(unsigned char *pCount, unsigned char *pTypeIsAperture, uint8_t devAddr)
{
int iTimeout;
unsigned char ucTemp;
#define MAX_TIMEOUT 50

  writeRegList(ucSPAD0,devAddr);
  writeReg(0x83, readReg(0x83,devAddr) | 0x04, devAddr);
  writeRegList(ucSPAD1,devAddr);
  iTimeout = 0;
  while(iTimeout < MAX_TIMEOUT)
  {
    if (readReg(0x83,devAddr) != 0x00) break;
    iTimeout++;
    HAL_Delay(5);
  }
  if (iTimeout == MAX_TIMEOUT)
  {
    fprintf(stderr, "Timeout while waiting for SPAD info\n");
    return 0;
  }
  writeReg(0x83,0x01,devAddr);
  ucTemp = readReg(0x92,devAddr);
  *pCount = (ucTemp & 0x7f);
  *pTypeIsAperture = (ucTemp & 0x80);
  writeReg(0x81,0x00,devAddr);
  writeReg(0xff,0x06,devAddr);
  writeReg(0x83, readReg(0x83,devAddr) & ~0x04, devAddr);
  writeRegList(ucSPAD2,devAddr);

  return 1;
} /* getSpadInfo() */

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
static uint16_t encodeTimeout(uint16_t timeout_mclks)
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

static void getSequenceStepTimeouts(uint8_t enables, SequenceStepTimeouts * timeouts, uint8_t devAddr)
{
  timeouts->pre_range_vcsel_period_pclks = ((readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD,devAddr) +1) << 1);

  timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP,devAddr) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,devAddr));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = ((readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD,devAddr) +1) << 1);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,devAddr));

  if (enables & SEQUENCE_ENABLE_PRE_RANGE)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
} /* getSequenceStepTimeouts() */


// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
static int setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks, uint8_t devAddr)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  uint8_t enables;
  SequenceStepTimeouts timeouts;

  enables = readReg(SYSTEM_SEQUENCE_CONFIG,devAddr);
  getSequenceStepTimeouts(enables, &timeouts,devAddr);

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
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18,devAddr);
        break;

      case 14:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30,devAddr);
        break;

      case 16:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40,devAddr);
        break;

      case 18:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50,devAddr);
        break;

      default:
        // invalid period
        return 0;
    }
    writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08,devAddr);

    // apply new VCSEL period
    writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg,devAddr);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    writeReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks),devAddr);

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1),devAddr);

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10,devAddr);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08,devAddr);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02,devAddr);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C,devAddr);
        writeReg(0xFF, 0x01,devAddr);
        writeReg(ALGO_PHASECAL_LIM, 0x30,devAddr);
        writeReg(0xFF, 0x00,devAddr);
        break;

      case 10:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28,devAddr);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08,devAddr);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03,devAddr);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09,devAddr);
        writeReg(0xFF, 0x01,devAddr);
        writeReg(ALGO_PHASECAL_LIM, 0x20,devAddr);
        writeReg(0xFF, 0x00,devAddr);
        break;

      case 12:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38,devAddr);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08,devAddr);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03,devAddr);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08,devAddr);
        writeReg(0xFF, 0x01,devAddr);
        writeReg(ALGO_PHASECAL_LIM, 0x20,devAddr);
        writeReg(0xFF, 0x00,devAddr);
        break;

      case 14:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48,devAddr);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08,devAddr);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03,devAddr);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07,devAddr);
        writeReg(0xFF, 0x01,devAddr);
        writeReg(ALGO_PHASECAL_LIM, 0x20,devAddr);
        writeReg(0xFF, 0x00,devAddr);
        break;

      default:
        // invalid period
        return 0;
    }

    // apply new VCSEL period
    writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg,devAddr);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables & SEQUENCE_ENABLE_PRE_RANGE)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    encodeTimeout(new_final_range_timeout_mclks),devAddr);

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return 0;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(measurement_timing_budget_us,devAddr);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG,devAddr);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02,devAddr);
  performSingleRefCalibration(0x0,devAddr);
  writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config,devAddr);

  // VL53L0X_perform_phase_calibration() end

  return 1;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
static int setMeasurementTimingBudget(uint32_t budget_us, uint8_t devAddr)
{
uint32_t used_budget_us;
uint32_t final_range_timeout_us;
uint16_t final_range_timeout_mclks;

  uint8_t enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return 0; }

  used_budget_us = StartOverhead + EndOverhead;

  enables = readReg(SYSTEM_SEQUENCE_CONFIG,devAddr);
  getSequenceStepTimeouts(enables, &timeouts,devAddr);

  if (enables & SEQUENCE_ENABLE_TCC)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables & SEQUENCE_ENABLE_DSS)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables & SEQUENCE_ENABLE_MSRC)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables & SEQUENCE_ENABLE_PRE_RANGE)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables & SEQUENCE_ENABLE_FINAL_RANGE)
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
      return 0;
    }

    final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables & SEQUENCE_ENABLE_PRE_RANGE)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks),devAddr);

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return 1;
}

static uint32_t getMeasurementTimingBudget(uint8_t devAddr)
{
  uint8_t enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  enables = readReg(SYSTEM_SEQUENCE_CONFIG,devAddr);
  getSequenceStepTimeouts(enables, &timeouts,devAddr);

  if (enables & SEQUENCE_ENABLE_TCC)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables & SEQUENCE_ENABLE_DSS)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables & SEQUENCE_ENABLE_MSRC)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables & SEQUENCE_ENABLE_PRE_RANGE)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables & SEQUENCE_ENABLE_FINAL_RANGE)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

static int performSingleRefCalibration(uint8_t vhv_init_byte, uint8_t devAddr)
{
int iTimeout;
  writeReg(SYSRANGE_START, 0x01 | vhv_init_byte,devAddr); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  iTimeout = 0;
  while ((readReg(RESULT_INTERRUPT_STATUS,devAddr) & 0x07) == 0)
  {
    iTimeout++;
    HAL_Delay(5);
    if (iTimeout > 100) { return 0; }
  }

  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01,devAddr);

  writeReg(SYSRANGE_START, 0x00,devAddr);

  return 1;
} /* performSingleRefCalibration() */

//
// Initialize the vl53l0x
//
static int initSensor(int bLongRangeMode, uint8_t devAddr)
{
unsigned char spad_count=0, spad_type_is_aperture=0, ref_spad_map[6];
unsigned char ucFirstSPAD, ucSPADsEnabled;
int i;

// set 2.8V mode
  writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
  readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,devAddr) | 0x01,devAddr); // set bit 0
// Set I2C standard mode
  writeRegList(ucI2CMode,devAddr);
  stop_variable = readReg(0x91,devAddr);
  writeRegList(ucI2CMode2,devAddr);
// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  writeReg(REG_MSRC_CONFIG_CONTROL, readReg(REG_MSRC_CONFIG_CONTROL, devAddr) | 0x12, devAddr);
  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  writeReg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32,devAddr); // 0.25
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF,devAddr);
  getSpadInfo(&spad_count, &spad_type_is_aperture, devAddr);

  readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6,devAddr);
//printf("initial spad map: %02x,%02x,%02x,%02x,%02x,%02x\n", ref_spad_map[0], ref_spad_map[1], ref_spad_map[2], ref_spad_map[3], ref_spad_map[4], ref_spad_map[5]);
  writeRegList(ucSPAD,devAddr);
  ucFirstSPAD = (spad_type_is_aperture) ? 12: 0;
  ucSPADsEnabled = 0;
// clear bits for unused SPADs
  for (i=0; i<48; i++)
  {
    if (i < ucFirstSPAD || ucSPADsEnabled == spad_count)
    {
      ref_spad_map[i>>3] &= ~(1<<(i & 7));
    }
    else if (ref_spad_map[i>>3] & (1<< (i & 7)))
    {
      ucSPADsEnabled++;
    }
  } // for i
  writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6,devAddr);
//printf("final spad map: %02x,%02x,%02x,%02x,%02x,%02x\n", ref_spad_map[0],
//ref_spad_map[1], ref_spad_map[2], ref_spad_map[3], ref_spad_map[4], ref_spad_map[5]);

// load default tuning settings
  writeRegList(ucDefTuning,devAddr); // long list of magic numbers

// change some settings for long range mode
  if (bLongRangeMode)
  {
	writeReg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 13,devAddr); // 0.1
	setVcselPulsePeriod(VcselPeriodPreRange, 18, devAddr);
	setVcselPulsePeriod(VcselPeriodFinalRange, 14, devAddr);
  }

// set interrupt configuration to "new sample ready"
  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04,devAddr);
  writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH,devAddr) & ~0x10,devAddr); // active low
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01,devAddr);
  measurement_timing_budget_us = getMeasurementTimingBudget(devAddr);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xe8,devAddr);
  setMeasurementTimingBudget(measurement_timing_budget_us, devAddr);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01,devAddr);
  if (!performSingleRefCalibration(0x40,devAddr)) { return 0; }
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02,devAddr);
  if (!performSingleRefCalibration(0x00,devAddr)) { return 0; }
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xe8,devAddr);
  return 1;
} /* initSensor() */

uint16_t readRangeContinuousMillimeters(uint8_t devAddr)
{
int iTimeout = 0;
uint16_t range;

  while ((readReg(RESULT_INTERRUPT_STATUS,devAddr) & 0x07) == 0)
  {
    iTimeout++;
    HAL_Delay(5);
    if (iTimeout > 50)
    {
      return -1;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  range = readReg16(RESULT_RANGE_STATUS + 10,devAddr);

  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01,devAddr);

  return range;
}
//
// Read the current distance in mm
//
int tofReadDistance(uint8_t devAddr)
{
int iTimeout;

  writeReg(0x80, 0x01,devAddr);
  writeReg(0xFF, 0x01,devAddr);
  writeReg(0x00, 0x00,devAddr);
  writeReg(0x91, stop_variable,devAddr);
  writeReg(0x00, 0x01,devAddr);
  writeReg(0xFF, 0x00,devAddr);
  writeReg(0x80, 0x00,devAddr);

  writeReg(SYSRANGE_START, 0x01,devAddr);

  // "Wait until start bit has been cleared"
  iTimeout = 0;
  while (readReg(SYSRANGE_START,devAddr) & 0x01)
  {
    iTimeout++;
    HAL_Delay(5);
    if (iTimeout > 50)
    {
      return -1;
    }
  }

  return readRangeContinuousMillimeters(devAddr);

} /* tofReadDistance() */

int tofGetModel(int *model, int *revision,uint8_t devAddr)
{
	uint8_t datatmp,regAddr;

	if (model)
	{
		regAddr = REG_IDENTIFICATION_MODEL_ID;
        HAL_I2C_Master_Transmit(&hi2c1,devAddr,&regAddr,1,1000);
        HAL_I2C_Master_Receive(&hi2c1,devAddr+1,&datatmp,1,1000);
		*model = datatmp;
	}
	if (revision)
	{
		regAddr = REG_IDENTIFICATION_REVISION_ID;
		HAL_I2C_Master_Transmit(&hi2c1,devAddr,&regAddr,1,1000);
		HAL_I2C_Master_Receive(&hi2c1,devAddr+1,&datatmp,1,1000);

		*revision = datatmp;
	}
	return 1;

} /* tofGetModel() */
