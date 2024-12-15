#include "imu9dof.h"
#include "Wire.h"

// Variables to set sensor scales
uint8_t Gscale = GFS_245DPS; // gyro full scale
uint8_t Godr = GODR_238Hz;   // gyro data sample rate
uint8_t Gbw = GBW_med;       // gyro data bandwidth
uint8_t Ascale = AFS_2G;     // accel full scale
uint8_t Aodr = AODR_238Hz;   // accel data sample rate
uint8_t Abw = ABW_50Hz;      // accel data bandwidth
uint8_t Mscale = MFS_4G;     // mag full scale
uint8_t Modr = MODR_10Hz;    // mag data sample rate
uint8_t Mmode = MMode_HighPerformance;  // magnetometer operation mode
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Variables for raw critical values
int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
float   temperature;    //Gyroscope temperature
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

// Variables for sensor fusion
float GyroMeasError = PI * (40.0f / 180.0f);      // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

// Variables for integration
uint32_t delt_t = 0, count = 0, sumCount = 0;
float deltat = 0.0f, sum = 0.0f;                  
uint32_t lastUpdate = 0, firstUpdate = 0;         
uint32_t Now = 0;

// Output variables
float roll, pitch, yaw;

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire1.beginTransmission(address);   
  Wire1.write(subAddress);           
  Wire1.write(data);                 
  Wire1.endTransmission();           
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;    
  Wire1.beginTransmission(address);         
  Wire1.write(subAddress);                        
  Wire1.endTransmission(false);               
  Wire1.requestFrom(address, (size_t) 1);    
  data = Wire1.read();                      
  return data;                             
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire1.beginTransmission(address);   // Initialize the Tx buffer
  Wire1.write(subAddress);            // Put slave register address in Tx buffer
  //  Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire1.requestFrom(address, count);  // Read bytes from slave register address 
  //        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire1.available()) {
    dest[i++] = Wire1.read(); }         // Put read results in the Rx buffer
}

void softResetIMU()
{
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x0c);
}

void getAccelResolution()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11). 
    case AFS_2G:
      aRes = 2.0/32768.0;
      break;
    case AFS_16G:
      aRes = 16.0/32768.0;
      break;
    case AFS_4G:
      aRes = 4.0/32768.0;
      break;
    case AFS_8G:
      aRes = 8.0/32768.0;
      break;
  }
}

void getGyroResolution()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), and 2000 DPS  (11). 
    case GFS_245DPS:
      gRes = 245.0/32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0/32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0/32768.0;
      break;
  }
}

void getMagResolution()
{
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
    case MFS_4G:
      mRes = 4.0/32768.0;
      break;
    case MFS_8G:
      mRes = 8.0/32768.0;
      break;
    case MFS_12G:
      mRes = 12.0/32768.0;
      break;
    case MFS_16G:
      mRes = 16.0/32768.0;
      break;
  }
}

void selftestLSM9DS1()
{
  float accel_noST[3] = {0., 0., 0.}, accel_ST[3] = {0., 0., 0.};
  float gyro_noST[3] = {0., 0., 0.}, gyro_ST[3] = {0., 0., 0.};

  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  accelgyrocalLSM9DS1(gyro_noST, accel_noST);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x05); // enable gyro/accel self test
  accelgyrocalLSM9DS1(gyro_ST, accel_ST);

  float gyrodx = (gyro_ST[0] - gyro_noST[0]);
  float gyrody = (gyro_ST[1] - gyro_noST[1]);
  float gyrodz = (gyro_ST[2] - gyro_noST[2]);

  float accdx = 1000.*(accel_ST[0] - accel_noST[0]);
  float accdy = 1000.*(accel_ST[1] - accel_noST[1]);
  float accdz = 1000.*(accel_ST[2] - accel_noST[2]);

  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  delay(200);
}

void accelgyrocalLSM9DS1(float * dest1, float * dest2)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;

  // enable the 3-axes of the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
  // configure the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
  delay(200);
  // enable the three axes of the accelerometer 
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
  // configure the accelerometer-specify bandwidth selection with Abw
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
  delay(200);
  // enable block data update, allow auto-increment during multiple byte read
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);

  // First get gyro bias
  byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable gyro FIFO  
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &data[0]);
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += (int32_t) gyro_temp[1]; 
    gyro_bias[2] += (int32_t) gyro_temp[2]; 
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 

  dest1[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
  dest1[1] = (float)gyro_bias[1]*gRes;
  dest1[2] = (float)gyro_bias[2]*gRes;

  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable gyro FIFO  
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable gyro bypass mode

  // now get the accelerometer bias
  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable accel FIFO  
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable accel FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the accel data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}

  dest2[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
  dest2[1] = (float)accel_bias[1]*aRes;
  dest2[2] = (float)accel_bias[2]*aRes;

  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable accel FIFO  
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable accel bypass mode
}

void magcalLSM9DS1(float * dest1)
{
    uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

  // configure the magnetometer-enable temperature compensation of mag data
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  sample_count = 128;
  for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
  }

  //    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  //    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  //    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1]*mRes;   
  dest1[2] = (float) mag_bias[2]*mRes;          

  //write biases to accelerometermagnetometer offset registers as counts);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);

  Serial.println("Mag Calibration done!");
}

void initLSM9DS1()
{  
  // enable the 3-axes of the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
  // configure the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
  delay(200);
  // enable the three axes of the accelerometer 
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
  // configure the accelerometer-specify bandwidth selection with Abw
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
  delay(200);
  // enable block data update, allow auto-increment during multiple byte read
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);
  // configure the magnetometer-enable temperature compensation of mag data
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_TEMP_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a 16-bit signed value
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}


