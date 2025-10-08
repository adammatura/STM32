/* bmi088.h - BMI088 6-Axis IMU Driver (Accelerometer + Gyroscope) */

#ifndef BMI088_H
#define BMI088_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Accelerometer Registers */
#define BMI088_ACC_CHIP_ID          0x00  // Should return 0x1E
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03
#define BMI088_ACC_X_LSB            0x12
#define BMI088_ACC_X_MSB            0x13
#define BMI088_ACC_Y_LSB            0x14
#define BMI088_ACC_Y_MSB            0x15
#define BMI088_ACC_Z_LSB            0x16
#define BMI088_ACC_Z_MSB            0x17
#define BMI088_ACC_TEMP_MSB         0x22
#define BMI088_ACC_TEMP_LSB         0x23
#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41
#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E

/* Gyroscope Registers */
#define BMI088_GYRO_CHIP_ID         0x00  // Should return 0x0F
#define BMI088_GYRO_RATE_X_LSB      0x02
#define BMI088_GYRO_RATE_X_MSB      0x03
#define BMI088_GYRO_RATE_Y_LSB      0x04
#define BMI088_GYRO_RATE_Y_MSB      0x05
#define BMI088_GYRO_RATE_Z_LSB      0x06
#define BMI088_GYRO_RATE_Z_MSB      0x07
#define BMI088_GYRO_INT_STAT_1      0x0A
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_SOFTRESET       0x14
#define BMI088_GYRO_INT_CTRL        0x15
#define BMI088_GYRO_SELF_TEST       0x3C

/* SPI Read/Write Bits */
#define BMI088_SPI_READ_BIT         0x80
#define BMI088_SPI_WRITE_BIT        0x00

/* Chip IDs */
#define BMI088_ACC_CHIP_ID_VALUE    0x1E
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F

/* Accelerometer Range Settings */
#define BMI088_ACC_RANGE_3G         0x00  // ±3g
#define BMI088_ACC_RANGE_6G         0x01  // ±6g
#define BMI088_ACC_RANGE_12G        0x02  // ±12g
#define BMI088_ACC_RANGE_24G        0x03  // ±24g

/* Accelerometer ODR and Bandwidth */
#define BMI088_ACC_ODR_12_5         0x05
#define BMI088_ACC_ODR_25           0x06
#define BMI088_ACC_ODR_50           0x07
#define BMI088_ACC_ODR_100          0x08
#define BMI088_ACC_ODR_200          0x09
#define BMI088_ACC_ODR_400          0x0A
#define BMI088_ACC_ODR_800          0x0B
#define BMI088_ACC_ODR_1600         0x0C

#define BMI088_ACC_BWP_NORMAL       0x00
#define BMI088_ACC_BWP_OSR2         0x01
#define BMI088_ACC_BWP_OSR4         0x02

/* Accelerometer Power Modes */
#define BMI088_ACC_PWR_ACTIVE       0x00
#define BMI088_ACC_PWR_SUSPEND      0x03

#define BMI088_ACC_ENABLE           0x04
#define BMI088_ACC_DISABLE          0x00

/* Gyroscope Range Settings */
#define BMI088_GYRO_RANGE_2000      0x00  // ±2000°/s
#define BMI088_GYRO_RANGE_1000      0x01  // ±1000°/s
#define BMI088_GYRO_RANGE_500       0x02  // ±500°/s
#define BMI088_GYRO_RANGE_250       0x03  // ±250°/s
#define BMI088_GYRO_RANGE_125       0x04  // ±125°/s

/* Gyroscope Bandwidth Settings */
#define BMI088_GYRO_BW_2000_532     0x00
#define BMI088_GYRO_BW_2000_230     0x01
#define BMI088_GYRO_BW_1000_116     0x02
#define BMI088_GYRO_BW_400_47       0x03
#define BMI088_GYRO_BW_200_23       0x04
#define BMI088_GYRO_BW_100_12       0x05
#define BMI088_GYRO_BW_200_64       0x06
#define BMI088_GYRO_BW_100_32       0x07

/* Gyroscope Power Modes */
#define BMI088_GYRO_PM_NORMAL       0x00
#define BMI088_GYRO_PM_SUSPEND      0x80
#define BMI088_GYRO_PM_DEEP_SUSPEND 0x20

/* Soft Reset */
#define BMI088_SOFT_RESET_CMD       0xB6

/* Accelerometer Sensitivity (LSB/g) */
#define BMI088_ACC_SENS_3G          10920.0f
#define BMI088_ACC_SENS_6G          5460.0f
#define BMI088_ACC_SENS_12G         2730.0f
#define BMI088_ACC_SENS_24G         1365.0f

/* Gyroscope Sensitivity (LSB/°/s) */
#define BMI088_GYRO_SENS_2000       16.384f
#define BMI088_GYRO_SENS_1000       32.768f
#define BMI088_GYRO_SENS_500        65.536f
#define BMI088_GYRO_SENS_250        131.072f
#define BMI088_GYRO_SENS_125        262.144f

/* Enumerations */
typedef enum {
    BMI088_ACC_RANGE_3G_E = 0,
    BMI088_ACC_RANGE_6G_E,
    BMI088_ACC_RANGE_12G_E,
    BMI088_ACC_RANGE_24G_E
} BMI088_AccRange_t;

typedef enum {
    BMI088_GYRO_RANGE_2000_E = 0,
    BMI088_GYRO_RANGE_1000_E,
    BMI088_GYRO_RANGE_500_E,
    BMI088_GYRO_RANGE_250_E,
    BMI088_GYRO_RANGE_125_E
} BMI088_GyroRange_t;

/* Data Structures */
typedef struct {
    float x;
    float y;
    float z;
} BMI088_AccelData_t;

typedef struct {
    float x;
    float y;
    float z;
} BMI088_GyroData_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} BMI088_RawData_t;

/* Configuration Structure */
typedef struct {
    BMI088_AccRange_t acc_range;
    BMI088_GyroRange_t gyro_range;
    uint8_t acc_odr;
    uint8_t acc_bwp;
    uint8_t gyro_bw;
} BMI088_Config_t;

/* Handle Structure */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *acc_cs_port;
    uint16_t acc_cs_pin;
    GPIO_TypeDef *gyro_cs_port;
    uint16_t gyro_cs_pin;
    float acc_sensitivity;
    float gyro_sensitivity;
    BMI088_AccRange_t acc_range;
    BMI088_GyroRange_t gyro_range;
} BMI088_Handle_t;

/* Function Prototypes */
bool BMI088_Init(BMI088_Handle_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
                 GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin);

bool BMI088_ConfigureAccel(BMI088_Handle_t *dev, BMI088_AccRange_t range,
                           uint8_t odr, uint8_t bwp);
bool BMI088_ConfigureGyro(BMI088_Handle_t *dev, BMI088_GyroRange_t range,
                          uint8_t bandwidth);

bool BMI088_ReadAccelRaw(BMI088_Handle_t *dev, BMI088_RawData_t *data);
bool BMI088_ReadGyroRaw(BMI088_Handle_t *dev, BMI088_RawData_t *data);

bool BMI088_ReadAccel(BMI088_Handle_t *dev, BMI088_AccelData_t *data);
bool BMI088_ReadGyro(BMI088_Handle_t *dev, BMI088_GyroData_t *data);

bool BMI088_ReadTemperature(BMI088_Handle_t *dev, float *temp);

bool BMI088_SoftReset(BMI088_Handle_t *dev);

#endif /* BMI088_H */
