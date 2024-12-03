#include <I2C_8Bit.h>
#include <Wire.h>
#include "AccGyrCal.h"

void setup_FastOffsetCompensation(bool foc_gyr_en, uint8_t foc_acc_x, uint8_t foc_acc_y, uint8_t foc_acc_z)
{
  uint8_t conf_value;
  uint8_t reserved = 0x00;

  conf_value = (reserved << 7) | ( (uint8_t) foc_gyr_en << 6) | (foc_acc_x << 4) | (foc_acc_y << 2) | (foc_acc_z);

  I2C_8Bit_writeToModule(BMX160_ADDR, FOC_CONF, conf_value);
}

void start_FastOffsetCompensation()
{
  uint8_t status_Value;

  I2C_8Bit_writeToModule(BMX160_ADDR, CMD, CMD_START_FOC);

  // Wait for the FOC_RDY bit to turn 1
  while(! ((I2C_8Bit_readFromModule(BMX160_ADDR, STATUS) >> 3) & 0x01) );

  // Clear the STATUS register
  I2C_8Bit_writeToModule(BMX160_ADDR, STATUS, 0x00);

  //Clear FOC_EN
  I2C_8Bit_writeToModule(BMX160_ADDR, CMD, 0x00);
}

void get_OffsetValues(uint8_t *acc_off_x, uint8_t *acc_off_y, uint8_t *acc_off_z, uint16_t *gyr_off_x, uint16_t *gyr_off_y, uint16_t *gyr_off_z)
{
  *acc_off_x = I2C_8Bit_readFromModule(BMX160_ADDR, ACC_OFF_X);
  *acc_off_y = I2C_8Bit_readFromModule(BMX160_ADDR, ACC_OFF_Y);
  *acc_off_z = I2C_8Bit_readFromModule(BMX160_ADDR, ACC_OFF_Z);

  *gyr_off_x = ((I2C_8Bit_readFromModule(BMX160_ADDR, OFFSET_6) & 0x03) << 8) | I2C_8Bit_readFromModule(BMX160_ADDR, GYR_OFF_X);
  *gyr_off_y = ((I2C_8Bit_readFromModule(BMX160_ADDR, OFFSET_6) >> 2) & 0x03) << 8 | I2C_8Bit_readFromModule(BMX160_ADDR, GYR_OFF_Y);
  *gyr_off_z = ((I2C_8Bit_readFromModule(BMX160_ADDR, OFFSET_6) >> 4) & 0x03) << 8 | I2C_8Bit_readFromModule(BMX160_ADDR, GYR_OFF_Z);

}

void set_inlineCalibration(bool Value)
{
  uint8_t RegVal;
  RegVal = I2C_8Bit_readFromModule(BMX160_ADDR, OFFSET_6);

  if(Value)
  {
    RegVal = RegVal | (0x03 << 6);
    I2C_8Bit_writeToModule(BMX160_ADDR, OFFSET_6, RegVal);
  }
}