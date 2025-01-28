#include "imu9dof.h"
#include "Wire.h"

#include <math.h>
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>

#include <DFRobot_BMX160.h>
#include "bmxUtils.h"

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#include <ArduinoBLE.h>

#define SAMPLE_RATE (119.0f)
#define SEALEVELPRESSURE_HPA (1013.25)

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

// Output variables
extern float roll, pitch, yaw;

// Variables for step detection and step length estimation
double distance = 0.0f, WeinbergConstant = 0.738894f;
float steplength = 0.0f;
bool step = false;
float maxima = 1.0f;
float minima = 1.5f;
float local_maxima;
float local_minima;
const float threshold_max = 1.06f;
const float threshold_min = 0.95f;
unsigned long previousStepTime = 0, currentStepTime;
const long debounceInterval = 53;

// Variables for low-pass filter
const double fc = 19;
const double fs = SAMPLE_RATE;
const double fn = fc / (fs/2);
Timer<micros> timer = std::round(1e6 / fs);
auto filter = butter<4>(fn);

// Variables for filtered accelerometer output
double Accz;

// Variables for altitude and pressure
float airPressure;

// Instance of the BMX160 sensor
DFRobot_BMX160 bmx160;

// Instance of the BMP388 sensor
Adafruit_BMP3XX bmp;

// Delay variables
unsigned long previousMillis = 0;        
const long interval = 1000;  

int RSSI;

void setup()
{
  Wire1.begin();
  Serial.begin(115200);

  if(!BLE.begin())
  {
    Serial.println("Starting BLE Failed");
    while(1);
  }

  BLE.scan();

  // Initialize the BMX sensor and perform inline calibration 
  if (bmx160.begin() != true){
    Serial.println("init false");
    while(1);
  }
  bmx160.wakeUp();  // Enable the accelerometer and gyroscope

  // Fast offset compensation (BMX160 Accelerometer and Gyroscope)
  setup_FastOffsetCompensation(1, 3, 3, 1);
  start_FastOffsetCompensation();
  set_inlineCalibration(true);

  // Initialize the BMP388 sensor and set params (According to datasheet)
  bmp.begin_I2C(0x76);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_31);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  // Initialize the LSM9DS1 sensor and perform calibration
  softResetIMU();
  getAccelResolution();
  getGyroResolution();
  getMagResolution();
  selftestLSM9DS1();
  accelgyrocalLSM9DS1(gyroBias, accelBias);   // Make sure the device does not move during this phase
  magcalLSM9DS1(magBias);                     // Doing the 8 for local magnetic field correction
  initLSM9DS1();

  // Configure step detection and interrupt on the BMX160
  configure_stepDetection(0);     // 0-Normal Mode, 1-Sensitive Mode, 2-Robust Mode
  configure_StepInterrupt();

  delay(5000);  // Wait for some time
}

void loop()
{
  //================================================================== BLE part ==============================================================================//
  BLEDevice peripheral = BLE.available();

  if(peripheral)
  {
    if(peripheral.localName() == "Nano33IoT")
    {
      RSSI = peripheral.rssi();
    }
  }



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
    mx = (float)magCount[0]*mRes - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes - magBias[1];  
    mz = (float)magCount[2]*mRes - magBias[2];   
  }

  //============================================= Orientation Estimation using Madgwick Filter ======================================================================//

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


  //==================================================== Step length estimation and step detection =================================================================//

  // Low pass butterworth filter (fc = 19Hz)
  if(timer)
  {
    Accz = filter(az);
  }

  // Finding local maxima and minima of the vertical acceleration for step-length estimation
  // Set thresholds according to person's build
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
    }
  }

  currentStepTime = millis();

  // Check for step (debouncing - wait after first step detected for 62 ms and discard the step detector output)
  if((display_InterruptStatus() & 0x01 == 0x01) && (currentStepTime - previousStepTime > debounceInterval))
  {
    previousStepTime = currentStepTime;

    step = true;
    steplength = CalculateStepLength();
  }
  else
  {
    step = false;
  }


  // =================================================================== Estimate Altitude and air pressure ============================================================== //
  
  // Poll this once every 1 seconds to avoid blocking for the yaw algorithm
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    airPressure = bmp.readPressure();
  }


  // =================================================================== Printing area ====================================================================================//

  // Print out all data - to send to a different device
  
  if(step)
  { Serial.print(step); Serial.print(", ");
    Serial.print(radians(yaw));  Serial.print(", ");
    Serial.print(steplength); Serial.print(", ");
    Serial.print(airPressure); Serial.print(", ");
    Serial.println(RSSI);
  } 

  /*
  Serial.print(step); Serial.print(", ");
  Serial.print(radians(yaw));  Serial.print(", ");
  Serial.print(steplength); Serial.print(", ");
  Serial.println(airPressure);
  */

  // Stop and reinitate scan to restart communication when in vicinity of Peripheral
  BLE.stopScan();
  BLE.scan();

}

float CalculateStepLength()
{
  return (WeinbergConstant * sqrt(sqrt(local_maxima - local_minima)));
}
