#ifndef __ACCGYRCAL_H__
#define __ACCGYRCAL_H__

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

void setup_FastOffsetCompensation(bool foc_gyr_en, uint8_t foc_acc_x, uint8_t foc_acc_y, uint8_t foc_acc_z);
void start_FastOffsetCompensation();
void get_OffsetValues(uint8_t *acc_off_x, uint8_t *acc_off_y, uint8_t *acc_off_z, uint16_t *gyr_off_x, uint16_t *gyr_off_y, uint16_t *gyr_off_z);
void set_inlineCalibration(bool Value);

#endif