#include <Adafruit_Sensor.h>
#include <DPEng_BMX160.h>
#include "AccGyrCal.h"

DPEng_BMX160 dpEng = DPEng_BMX160(0x160A, 0x160B, 0x160C);
uint8_t accOffsetX, accOffsetY, accOffsetZ;
uint16_t gyrOffsetX, gyrOffsetY, gyrOffsetZ;

void setup()
{
  Serial.begin(115200);

  while(!Serial)
  {
    delay(1);
  }

  
  if(!dpEng.begin(BMX160_ACCELRANGE_4G, GYRO_RANGE_250DPS))
  {
    Serial.println("Ooops, no BMX160 detected ... Check your wiring!");
    while(1);
  }

  delay(200);

  setup_FastOffsetCompensation(1, 3, 3, 3);
  start_FastOffsetCompensation();
  
  get_OffsetValues(&accOffsetX, &accOffsetY, &accOffsetZ, &gyrOffsetX, &gyrOffsetY, &gyrOffsetZ);

  Serial.print(accOffsetX); Serial.print(", ");
  Serial.print(accOffsetY); Serial.print(", ");
  Serial.print(accOffsetZ); Serial.println("");

  Serial.print(gyrOffsetX); Serial.print(", ");
  Serial.print(gyrOffsetY); Serial.print(", ");
  Serial.print(gyrOffsetZ); Serial.println("");

  set_inlineCalibration(true);

}

void loop()
{

  sensors_event_t aevent, gevent, mevent;

  dpEng.getEvent(&aevent, &gevent, &mevent);

  

  //Serial.print(aevent.acceleration.x);  Serial.print(", ");
  //Serial.print(aevent.acceleration.y);  Serial.print(", ");
  //Serial.print(aevent.acceleration.z);  Serial.println("");
  
  delay(100);
}