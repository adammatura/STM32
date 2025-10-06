/* ms5611.c - MS5611 Barometric Pressure Sensor Driver Implementation */

#include "ms5611.h"
#include <string.h>

/* Private Functions */
static inline void MS5611_CS_Low(MS5611_Handle_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void MS5611_CS_High(MS5611_Handle_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static bool MS5611_WriteCommand(MS5611_Handle_t *dev, uint8_t cmd) {
    MS5611_CS_Low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    MS5611_CS_High(dev);
    return (status == HAL_OK);
}

static bool MS5611_ReadData(MS5611_Handle_t *dev, uint8_t *data, uint16_t len) {
    MS5611_CS_Low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Receive(dev->hspi, data, len, 100);
    MS5611_CS_High(dev);
    return (status == HAL_OK);
}

static uint16_t MS5611_ReadPROM(MS5611_Handle_t *dev, uint8_t addr) {
    uint8_t cmd = MS5611_CMD_PROM_READ | (addr << 1);
    uint8_t data[2];

    MS5611_CS_Low(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, data, 2, 100);
    MS5611_CS_High(dev);

    return (data[0] << 8) | data[1];
}

static uint8_t MS5611_GetOSRCommand(MS5611_OSR_t osr, bool is_pressure) {
    uint8_t base = is_pressure ? MS5611_CMD_CONV_D1_256 : MS5611_CMD_CONV_D2_256;
    return base + (osr * 2);
}

static uint16_t MS5611_GetConversionTime(MS5611_OSR_t osr) {
    const uint16_t times[] = {1, 2, 3, 5, 10};  // Conversion times in ms
    return times[osr];
}

/* Public Functions */
bool MS5611_Init(MS5611_Handle_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin, MS5611_OSR_t osr) {
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->osr = osr;

    MS5611_CS_High(dev);
    HAL_Delay(10);

    if (!MS5611_Reset(dev)) {
        return false;
    }

    HAL_Delay(100);  // Wait for reset to complete

    if (!MS5611_ReadCalibration(dev)) {
        return false;
    }

    return true;
}

bool MS5611_Reset(MS5611_Handle_t *dev) {
    return MS5611_WriteCommand(dev, MS5611_CMD_RESET);
}

bool MS5611_ReadCalibration(MS5611_Handle_t *dev) {
    dev->calib.C1 = MS5611_ReadPROM(dev, 1);
    dev->calib.C2 = MS5611_ReadPROM(dev, 2);
    dev->calib.C3 = MS5611_ReadPROM(dev, 3);
    dev->calib.C4 = MS5611_ReadPROM(dev, 4);
    dev->calib.C5 = MS5611_ReadPROM(dev, 5);
    dev->calib.C6 = MS5611_ReadPROM(dev, 6);

    // Basic validation
    if (dev->calib.C1 == 0 || dev->calib.C1 == 0xFFFF) {
        return false;
    }

    return true;
}

bool MS5611_ReadRawPressure(MS5611_Handle_t *dev, uint32_t *pressure) {
    uint8_t cmd = MS5611_GetOSRCommand(dev->osr, true);

    if (!MS5611_WriteCommand(dev, cmd)) {
        return false;
    }

    HAL_Delay(MS5611_GetConversionTime(dev->osr));

    uint8_t data[3];
    MS5611_CS_Low(dev);
    uint8_t adc_cmd = MS5611_CMD_ADC_READ;
    HAL_SPI_Transmit(dev->hspi, &adc_cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, data, 3, 100);
    MS5611_CS_High(dev);

    *pressure = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    return true;
}

bool MS5611_ReadRawTemperature(MS5611_Handle_t *dev, uint32_t *temperature) {
    uint8_t cmd = MS5611_GetOSRCommand(dev->osr, false);

    if (!MS5611_WriteCommand(dev, cmd)) {
        return false;
    }

    HAL_Delay(MS5611_GetConversionTime(dev->osr));

    uint8_t data[3];
    MS5611_CS_Low(dev);
    uint8_t adc_cmd = MS5611_CMD_ADC_READ;
    HAL_SPI_Transmit(dev->hspi, &adc_cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, data, 3, 100);
    MS5611_CS_High(dev);

    *temperature = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    return true;
}

bool MS5611_Calculate(MS5611_Handle_t *dev, uint32_t D1, uint32_t D2,
                      float *pressure, float *temperature) {
    int32_t dT, TEMP;
    int64_t OFF, SENS, P;

    // Calculate temperature
    dT = D2 - ((int32_t)dev->calib.C5 << 8);
    TEMP = 2000 + ((int64_t)dT * dev->calib.C6 >> 23);

    // Calculate temperature compensated pressure
    OFF = ((int64_t)dev->calib.C2 << 16) + (((int64_t)dev->calib.C4 * dT) >> 7);
    SENS = ((int64_t)dev->calib.C1 << 15) + (((int64_t)dev->calib.C3 * dT) >> 8);

    // Second order temperature compensation
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;

    if (TEMP < 2000) {
        T2 = ((int64_t)dT * dT) >> 31;
        OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 2;

        if (TEMP < -1500) {
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) >> 1;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    P = (((D1 * SENS) >> 21) - OFF) >> 15;

    *temperature = TEMP / 100.0f;  // Temperature in °C
    *pressure = P / 100.0f;        // Pressure in mbar

    return true;
}

bool MS5611_ReadSensor(MS5611_Handle_t *dev, float *pressure, float *temperature) {
    uint32_t D1, D2;

    if (!MS5611_ReadRawPressure(dev, &D1)) {
        return false;
    }

    if (!MS5611_ReadRawTemperature(dev, &D2)) {
        return false;
    }

    return MS5611_Calculate(dev, D1, D2, pressure, temperature);
}
