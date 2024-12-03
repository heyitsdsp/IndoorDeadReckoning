#include <DFRobot_BMX160.h>
#include <I2C_8Bit.h>
#include <Wire.h>

#define BMX160_ADDR       0x68
#define STEP_CONF_ADDR0   0x7A
#define STEP_CONF_ADDR1   0x7B 
#define STEP_MODE_NORM0   0x15
#define STEP_MODE_NORM1   0x0B
#define STEP_MODE_SENS0   0x2D
#define STEP_MODE_SENS1   0x08
#define STEP_MODE_ROB0    0x1D
#define STEP_MODE_ROB1    0x0F

DFRobot_BMX160 bmx160;

namespace EnumGyro
{
  enum eGyroRange_t {
    eGyroRange_2000DPS,
    eGyroRange_1000DPS,
    eGyroRange_500DPS,
    eGyroRange_250DPS,
    eGyroRange_125DPS
  };
};

namespace EnumAccel
{
  enum eAccelRange_t {
    eAccelRange_2G,
    eAccelRange_4G,
    eAccelRange_8G,
    eAccelRange_16G
  };
};

void setup()
{
  Serial.begin(115200);
  delay(100);

  if(bmx160.begin() != true)
  {
    Serial.println("Couldn't start up the BMX160 sensor!");
    while(1);   // hang the program
  }

  bmx160.softReset();   // Soft Reset the sensor
  bmx160.wakeUp();      // Wakeup the sensor
   
  bmx160.setGyroRange(eGyroRange_t :: eGyroRange_250DPS);
  bmx160.setAccelRange(eAccelRange_t :: eAccelRange_2G);

  I2C_8Bit_begin();

  delay(100);   // Delay to allow for some setup time
}

void loop()
{
  uint8_t steps[2];
  uint16_t step_count = 0;

  I2C_8Bit_writeToModule(BMX160_ADDR, STEP_CONF_ADDR0, STEP_MODE_SENS0);
  I2C_8Bit_writeToModule(BMX160_ADDR, STEP_CONF_ADDR1, STEP_MODE_SENS1);

  steps[0] = I2C_8Bit_readFromModule(BMX160_ADDR, 0x78);
  steps[1] = I2C_8Bit_readFromModule(BMX160_ADDR, 0x79);

  step_count = (steps[1] << 8) | (steps[0]); 

  Serial.println(step_count);

  delay(150);
  
}