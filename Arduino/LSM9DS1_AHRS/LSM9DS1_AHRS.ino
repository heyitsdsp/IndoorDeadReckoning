#include "imu9dof.h"
#include "Wire.h"
#include <math.h>
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

float accelBias[3] = {0, 0, 0}, gyroBias[3] = {0, 0, 0},  magBias[3] = {0, 0, 0}; // Offsets for accelerometer, gyroscope and magnetometer

// Variables to set sensor scales
extern float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Variables for raw critical values
extern int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
extern float   temperature;    //Gyroscope temperature
extern float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
extern float q[4];    // vector to hold quaternion
float q_inverse[4];
float q_magRot[4];
float magRot[4];

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

// Variables for filtering
const double fc = 19;
const double fs = 119.00;
const double fn = fc / (fs/2);
uint16_t steps = 0;
bool step = false;

// Variables for step counting
float Accz;               // Filtered z axis acceleration
float maxima = 1.0f;
float minima = 1.5f;
float local_maxima;
float local_minima;
const float threshold_max = 1.06f;
const float threshold_min = 0.95f;

double distance = 0.0f, WeinbergConstant = 0.738894f;
float steplength = 0.0f;

Timer<micros> timer = std::round(1e6 / fs);
auto filter = butter<2>(fn);

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

  delay(3000);

  bmp.begin_I2C(0x76);

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  initLSM9DS1();
}

void loop()
{
  step = false;

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
    mx = (float)magCount[0]*mRes;  //- magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes;  //- magBias[1];  
    mz = (float)magCount[2]*mRes;  //- magBias[2];   
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

  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI; 
  yaw   -= 2.96f; // Declination at Aachen
  roll  *= 180.0f / PI;
  // Convert yaw to normal compass degrees   
  if (yaw < 0) yaw += 360.0;
  if (yaw >= 360.0) yaw -= 360.0;


  /* ===================================== STEP LENGTH AND DISTANCE ESTIMATION ==================================== */ 
  if(timer)
  {
    Accz = filter(az);
  }

  if(Accz > threshold_max)
  {
    if(Accz >= maxima)
    {
      maxima = Accz;
    }
    else
    {
      local_maxima = maxima;
      maxima = 1.0f;
      steps += 1;
    }
  }

  if(Accz < threshold_min)
  {
    if(Accz <= minima)
    {
      minima = Accz;
    }
    else
    {
      local_minima = minima;
      minima = 1.5f;
      
      if(! isnan(CalculateStepLength(&local_maxima, &local_minima)))
      {
        step = true;
        distance += CalculateStepLength(&local_maxima, &local_minima);
        steplength = CalculateStepLength(&local_maxima, &local_minima);
      }
    }
  }


  /* ============================================ ALTITUDE ESTIMATION ========================================== */
  
  bmp.performReading();
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);


  /* ============================================ PRINTING OUTPUTS TO SERIAL ====================================== */ 
  
  Serial.print(step); Serial.print(", ");
  Serial.print(yaw); Serial.print(", ");
  Serial.print(steplength); Serial.print(", ");
  Serial.println(altitude);

  delay(50);
}

float CalculateStepLength(float* loc_max, float* loc_min)
{
  float step_length = WeinbergConstant * sqrt(sqrt(*loc_max - *loc_min));

  return step_length;
}