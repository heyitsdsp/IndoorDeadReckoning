#ifndef __BMXUTILS_H__
#define __BMXUTILS_H__

#define BMX160_ADDR       0x68
#define FOC_CONF          0x69
#define ACC_OFF_X         0x71
#define ACC_OFF_Y         0x72
#define ACC_OFF_Z         0x73
#define GYR_OFF_X         0x74
#define GYR_OFF_Y         0x75
#define GYR_OFF_Z         0x76
#define OFFSET_6          0x77
#define CMD               0x7E
#define STATUS            0x1B
#define CMD_START_FOC     0x03
#define STEP_CONF_ADDR0   0x7A
#define STEP_CONF_ADDR1   0x7B 
#define STEP_MODE_NORM0   0x15
#define STEP_MODE_NORM1   0x0B
#define STEP_MODE_SENS0   0x2D
#define STEP_MODE_SENS1   0x08
#define STEP_MODE_ROB0    0x1D
#define STEP_MODE_ROB1    0x0F
#define INT_EN_ADDR0      0x50
#define INT_EN_ADDR1      0x51
#define INT_EN_ADDR2      0x52
#define INT_OUT_CTRL      0x53
#define INT_LATCH         0x54
#define INT_MAP_ADDR0     0x55
#define INT_MAP_ADDR1     0x56
#define INT_MAP_ADDR2     0x57
#define INT_STATUS_ADDR0  0x1C


void setup_FastOffsetCompensation(bool foc_gyr_en, uint8_t foc_acc_x, uint8_t foc_acc_y, uint8_t foc_acc_z);
void start_FastOffsetCompensation();
void get_OffsetValues(uint8_t *acc_off_x, uint8_t *acc_off_y, uint8_t *acc_off_z, uint16_t *gyr_off_x, uint16_t *gyr_off_y, uint16_t *gyr_off_z);
void set_inlineCalibration(bool Value);
void configure_stepDetection(uint8_t step_val);
void configure_StepInterrupt();
void clear_InterruptStatus();
uint8_t display_InterruptStatus();

#endif