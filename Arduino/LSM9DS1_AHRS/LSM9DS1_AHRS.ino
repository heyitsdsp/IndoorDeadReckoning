#include "imu9dof.h"
#include "Wire.h"

float accelBias[3] = {0, 0, 0}, gyroBias[3] = {0, 0, 0},  magBias[3] = {0, 0, 0}; // Offsets for accelerometer, gyroscope and magnetometer

// Variables to set sensor scales
extern float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Variables for raw critical values
extern int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
extern float   temperature;    //Gyroscope temperature
extern float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
extern float q[4];    // vector to hold quaternion

// Variables for sensor fusion
extern float GyroMeasError;      // gyroscope measurement error in rads/s (start at 40 deg/s)
extern float GyroMeasDrift;      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
extern float beta;   // compute beta
extern float zeta;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

// Variables for integration
extern uint32_t delt_t, count, sumCount;
extern float deltat, sum;                  
extern uint32_t lastUpdate, firstUpdate;         
extern uint32_t Now;

// Tilt compensation variables
float Xm, Ym;

// Output variables
extern float roll, pitch, yaw;

void setup()
{
  Wire1.begin();
  Serial.begin(9600);
  softResetIMU();
  getAccelResolution();
  getGyroResolution();
  getMagResolution();
  selftestLSM9DS1();
  accelgyrocalLSM9DS1(gyroBias, accelBias);
  magcalLSM9DS1(magBias);
  initLSM9DS1();
}

void loop()
{
    if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x01) {  // check if new accel data is ready  
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2]; 
  } 

  if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x02) {  // check if new gyro data is ready  
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   
  }

  if (readByte(LSM9DS1M_ADDRESS, LSM9DS1M_STATUS_REG_M) & 0x08) {  // check if new mag data is ready  
    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes; // - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes; // - magBias[1];  
    mz = (float)magCount[2]*mRes; // - magBias[2];   
  }

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx, my, mz);

  delt_t = millis() - count;

  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

  // Tilt compensation
  Xm = mx*cos(pitch) - my*sin(roll)*sin(pitch) + mz*cos(roll)*sin(pitch);
  Ym = my*cos(roll) + mz*sin(roll);

  yaw = atan2(Ym,Xm);
  //

  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI; 
  yaw   -= 2.96f; // Declination at Aachen
  roll  *= 180.0f / PI;
  // Convert yaw to normal compass degrees   
  if (yaw < 0) yaw += 360.0;
  if (yaw >= 360.0) yaw -= 360.0;

  Serial.print("Yaw, Pitch, Roll: ");
  Serial.print(yaw, 2);
  Serial.print(", ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);

  delay(50);
}