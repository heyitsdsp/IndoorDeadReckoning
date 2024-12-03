#include <DPEng_BMX160.h>
#include "AccGyrCal.h"
#include <Wire.h>
#include <I2C_8Bit.h>
#include <math.h>

#define DEGREES_TO_RADIANS  (0.00174533F)
#define RADIANS_TO_DEGREES  (57.2957795F)

DPEng_BMX160 dpEng = DPEng_BMX160(0x160A, 0x160B, 0x160C);

double dt, millisOld;

float PitchRad, RollRad;

float AccBiases[3] = {-0.152683F, -0.036126F, 0.049236F};

float AccScaleFactors[3][3] = {{1.164210F, -0.015121F, -0.057322F},
                         {-0.015121F, 1.010395F, -0.007279F},
                         {-0.057322F, -0.007279F, 1.067525F}};

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -71.050334F, -482.861443F, 0.813164F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 0.055750F, 0.000394F, -0.000334F},
                                    { 0.000394F, 0.067771F, -0.000011F},
                                    {-0.000334F, -0.000011F,  0.095063F} };

float RollAcc, PitchAcc;
float Roll = 0, Pitch = 0, Yaw = 0;

void CalibrateAccReadings(float *ax, float *ay, float *az)
{
  float x = *ax - AccBiases[0];
  float y = *ay - AccBiases[1];
  float z = *az - AccBiases[2];

  *ax = x * AccScaleFactors[0][0] + y * AccScaleFactors[0][1] + z * AccScaleFactors[0][2];
  *ay = x * AccScaleFactors[1][0] + y * AccScaleFactors[1][1] + z * AccScaleFactors[1][2];
  *az = x * AccScaleFactors[2][0] + y * AccScaleFactors[2][1] + z * AccScaleFactors[2][2];
}

void setup()
{
  Serial.begin(115200);

  if(!dpEng.begin(BMX160_ACCELRANGE_4G, GYRO_RANGE_250DPS))
  {
    Serial.println("Ooops, no sensor detected ... Check your wiring!");
    while(1);
  }

  setup_FastOffsetCompensation(true, 3, 3, 1);
  start_FastOffsetCompensation();
  set_inlineCalibration(true);

  millisOld = millis();

}

void loop(void)
{
	sensors_event_t accel_event;
	sensors_event_t gyro_event;
	sensors_event_t mag_event; 

  dt = (millis() - millisOld) / 1000;
  millisOld = millis();

    // Get new data samples
	dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

  float accx = accel_event.acceleration.x / 9.80665F;
  float accy = accel_event.acceleration.y / 9.80665F;
  float accz = accel_event.acceleration.z / 9.80665F;

  //CalibrateAccReadings(&accx, &accy, &accz);

  PitchAcc = - atan2(accx, accz) * RADIANS_TO_DEGREES;
  RollAcc = - atan2(accy, accz) * RADIANS_TO_DEGREES;
  
  Pitch = (Pitch + gyro_event.gyro.y * dt) * 0.95 + PitchAcc * 0.05;
  Roll =  (Roll + gyro_event.gyro.x * dt) * 0.70 + RollAcc * 0.30;

  PitchRad = Pitch * DEGREES_TO_RADIANS;
  RollRad = Roll * DEGREES_TO_RADIANS;

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  //mx = mx * cos(PitchRad) - my * sin(RollRad) * sin(PitchRad) + mz * cos(RollRad) * sin(PitchRad);
  //my = my * cos(RollRad) + mz * sin(RollRad);

  Yaw = degrees(atan2(my, mx));

  Serial.println(Yaw);
  
  delay(100);

}
