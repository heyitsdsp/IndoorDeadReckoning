#include "DFRobot_BMP388.h"
#include "DFRobot_BMP388_I2C.h"
#include "Wire.h"
#include "SPI.h"
#include "math.h"
#include "bmp3_defs.h"

/*If there is no need to calibrate altitude, comment this line*/
//#define CALIBRATE_Altitude

DFRobot_BMP388_I2C bmp388;

float seaLevel;

void setup(){

  Serial.begin(115200);

  bmp388.set_iic_addr(BMP3_I2C_ADDR_PRIM);
  /* Initialize bmp388*/
  while(bmp388.begin()){
    Serial.println("Initialize error!");
    delay(1000);
  }

  delay(100);
  seaLevel = bmp388.readSeaLevel(525.0);
  Serial.print("seaLevel : ");
  Serial.print(seaLevel);
  Serial.println(" Pa");
}

void loop(){
  #ifdef CALIBRATE_Altitude
  /* Read the calibrated altitude */
  float altitude = bmp388.readCalibratedAltitude(seaLevel);
  Serial.print("calibrate Altitude : ");
  Serial.print(altitude);
  Serial.println(" m");
  #else
  /* Read the altitude */
  float altitude = bmp388.readAltitude();
  Serial.print("Altitude : ");
  Serial.print(altitude);
  Serial.println(" m");
  #endif
  delay(100);
}