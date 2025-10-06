/* ms5611.h - MS5611 Barometric Pressure Sensor Driver */

#ifndef MS5611_H
#define MS5611_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* MS5611 Commands */
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_256  0x40  // Pressure OSR=256
#define MS5611_CMD_CONV_D1_512  0x42  // Pressure OSR=512
#define MS5611_CMD_CONV_D1_1024 0x44  // Pressure OSR=1024
#define MS5611_CMD_CONV_D1_2048 0x46  // Pressure OSR=2048
#define MS5611_CMD_CONV_D1_4096 0x48  // Pressure OSR=4096

#define MS5611_CMD_CONV_D2_256  0x50  // Temperature OSR=256
#define MS5611_CMD_CONV_D2_512  0x52  // Temperature OSR=512
#define MS5611_CMD_CONV_D2_1024 0x54  // Temperature OSR=1024
#define MS5611_CMD_CONV_D2_2048 0x56  // Temperature OSR=2048
#define MS5611_CMD_CONV_D2_4096 0x58  // Temperature OSR=4096

#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ    0xA0  // Base address for PROM

/* Oversampling Ratio */
typedef enum {
    MS5611_OSR_256  = 0,
    MS5611_OSR_512  = 1,
    MS5611_OSR_1024 = 2,
    MS5611_OSR_2048 = 3,
    MS5611_OSR_4096 = 4
} MS5611_OSR_t;

/* Calibration Data Structure */
typedef struct {
    uint16_t C1;  // Pressure sensitivity
    uint16_t C2;  // Pressure offset
    uint16_t C3;  // Temperature coefficient of pressure sensitivity
    uint16_t C4;  // Temperature coefficient of pressure offset
    uint16_t C5;  // Reference temperature
    uint16_t C6;  // Temperature coefficient of the temperature
} MS5611_CalibData_t;

/* MS5611 Handle Structure */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    MS5611_CalibData_t calib;
    MS5611_OSR_t osr;
} MS5611_Handle_t;

/* Function Prototypes */
bool MS5611_Init(MS5611_Handle_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin, MS5611_OSR_t osr);
bool MS5611_Reset(MS5611_Handle_t *dev);
bool MS5611_ReadCalibration(MS5611_Handle_t *dev);
bool MS5611_ReadRawPressure(MS5611_Handle_t *dev, uint32_t *pressure);
bool MS5611_ReadRawTemperature(MS5611_Handle_t *dev, uint32_t *temperature);
bool MS5611_Calculate(MS5611_Handle_t *dev, uint32_t D1, uint32_t D2,
                      float *pressure, float *temperature);
bool MS5611_ReadSensor(MS5611_Handle_t *dev, float *pressure, float *temperature);

#endif /* MS5611_H */
