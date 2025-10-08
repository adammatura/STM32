/* bmi088.c - BMI088 6-Axis IMU Driver Implementation */

#include "bmi088.h"

/* Private Functions */

static inline void BMI088_AccelCS_Low(BMI088_Handle_t *dev) {
    HAL_GPIO_WritePin(dev->acc_cs_port, dev->acc_cs_pin, GPIO_PIN_RESET);
}

static inline void BMI088_AccelCS_High(BMI088_Handle_t *dev) {
    HAL_GPIO_WritePin(dev->acc_cs_port, dev->acc_cs_pin, GPIO_PIN_SET);
}

static inline void BMI088_GyroCS_Low(BMI088_Handle_t *dev) {
    HAL_GPIO_WritePin(dev->gyro_cs_port, dev->gyro_cs_pin, GPIO_PIN_RESET);
}

static inline void BMI088_GyroCS_High(BMI088_Handle_t *dev) {
    HAL_GPIO_WritePin(dev->gyro_cs_port, dev->gyro_cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Write to accelerometer register
 */
static bool BMI088_AccelWriteReg(BMI088_Handle_t *dev, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg & 0x7F, value};  // Clear read bit

    BMI088_AccelCS_Low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, data, 2, 100);
    BMI088_AccelCS_High(dev);

    // Wait at least 2us for internal synchronization
    HAL_Delay(1);

    return (status == HAL_OK);
}

/**
 * @brief Read from accelerometer register
 * NOTE: Accelerometer has a dummy byte on first read!
 */
static bool BMI088_AccelReadReg(BMI088_Handle_t *dev, uint8_t reg, uint8_t *buffer, uint16_t len) {
    uint8_t cmd = reg | BMI088_SPI_READ_BIT;
    uint8_t dummy;

    BMI088_AccelCS_Low(dev);

    // Send register address
    if (HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100) != HAL_OK) {
        BMI088_AccelCS_High(dev);
        return false;
    }

    // Read and discard dummy byte
    if (HAL_SPI_Receive(dev->hspi, &dummy, 1, 100) != HAL_OK) {
        BMI088_AccelCS_High(dev);
        return false;
    }

    // Read actual data
    HAL_StatusTypeDef status = HAL_SPI_Receive(dev->hspi, buffer, len, 100);

    BMI088_AccelCS_High(dev);

    return (status == HAL_OK);
}

/**
 * @brief Write to gyroscope register
 */
static bool BMI088_GyroWriteReg(BMI088_Handle_t *dev, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg & 0x7F, value};  // Clear read bit

    BMI088_GyroCS_Low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, data, 2, 100);
    BMI088_GyroCS_High(dev);

    return (status == HAL_OK);
}

/**
 * @brief Read from gyroscope register
 * NOTE: Gyroscope does NOT have dummy byte
 */
static bool BMI088_GyroReadReg(BMI088_Handle_t *dev, uint8_t reg, uint8_t *buffer, uint16_t len) {
    uint8_t cmd = reg | BMI088_SPI_READ_BIT;

    BMI088_GyroCS_Low(dev);

    // Send register address
    if (HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100) != HAL_OK) {
        BMI088_GyroCS_High(dev);
        return false;
    }

    // Read data (no dummy byte for gyro)
    HAL_StatusTypeDef status = HAL_SPI_Receive(dev->hspi, buffer, len, 100);

    BMI088_GyroCS_High(dev);

    return (status == HAL_OK);
}

/**
 * @brief Get accelerometer sensitivity based on range
 */
static float BMI088_GetAccelSensitivity(BMI088_AccRange_t range) {
    switch(range) {
        case BMI088_ACC_RANGE_3G_E:  return BMI088_ACC_SENS_3G;
        case BMI088_ACC_RANGE_6G_E:  return BMI088_ACC_SENS_6G;
        case BMI088_ACC_RANGE_12G_E: return BMI088_ACC_SENS_12G;
        case BMI088_ACC_RANGE_24G_E: return BMI088_ACC_SENS_24G;
        default: return BMI088_ACC_SENS_6G;
    }
}

/**
 * @brief Get gyroscope sensitivity based on range
 */
static float BMI088_GetGyroSensitivity(BMI088_GyroRange_t range) {
    switch(range) {
        case BMI088_GYRO_RANGE_2000_E: return BMI088_GYRO_SENS_2000;
        case BMI088_GYRO_RANGE_1000_E: return BMI088_GYRO_SENS_1000;
        case BMI088_GYRO_RANGE_500_E:  return BMI088_GYRO_SENS_500;
        case BMI088_GYRO_RANGE_250_E:  return BMI088_GYRO_SENS_250;
        case BMI088_GYRO_RANGE_125_E:  return BMI088_GYRO_SENS_125;
        default: return BMI088_GYRO_SENS_2000;
    }
}

/* Public Functions */

/**
 * @brief Initialize BMI088
 */
bool BMI088_Init(BMI088_Handle_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
                 GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin) {

    dev->hspi = hspi;
    dev->acc_cs_port = acc_cs_port;
    dev->acc_cs_pin = acc_cs_pin;
    dev->gyro_cs_port = gyro_cs_port;
    dev->gyro_cs_pin = gyro_cs_pin;

    // Set CS pins high
    BMI088_AccelCS_High(dev);
    BMI088_GyroCS_High(dev);

    HAL_Delay(10);

    // Soft reset accelerometer
    if (!BMI088_AccelWriteReg(dev, BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET_CMD)) {
        return false;
    }
    HAL_Delay(50);  // Wait for reset

    // Soft reset gyroscope
    if (!BMI088_GyroWriteReg(dev, BMI088_GYRO_SOFTRESET, BMI088_SOFT_RESET_CMD)) {
        return false;
    }
    HAL_Delay(50);  // Wait for reset

    // Check accelerometer chip ID
    uint8_t acc_id;
    if (!BMI088_AccelReadReg(dev, BMI088_ACC_CHIP_ID, &acc_id, 1)) {
        return false;
    }
    if (acc_id != BMI088_ACC_CHIP_ID_VALUE) {
        return false;
    }

    // Check gyroscope chip ID
    uint8_t gyro_id;
    if (!BMI088_GyroReadReg(dev, BMI088_GYRO_CHIP_ID, &gyro_id, 1)) {
        return false;
    }
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return false;
    }

    // Enable accelerometer (must write 0x04 to ACC_PWR_CTRL)
    if (!BMI088_AccelWriteReg(dev, BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE)) {
        return false;
    }
    HAL_Delay(5);  // Wait at least 5ms after enabling

    // Set accelerometer to active mode
    if (!BMI088_AccelWriteReg(dev, BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE)) {
        return false;
    }
    HAL_Delay(5);

    // Configure with default settings
    if (!BMI088_ConfigureAccel(dev, BMI088_ACC_RANGE_6G_E, BMI088_ACC_ODR_100, BMI088_ACC_BWP_NORMAL)) {
        return false;
    }

    if (!BMI088_ConfigureGyro(dev, BMI088_GYRO_RANGE_2000_E, BMI088_GYRO_BW_2000_532)) {
        return false;
    }

    return true;
}

/**
 * @brief Configure accelerometer
 */
bool BMI088_ConfigureAccel(BMI088_Handle_t *dev, BMI088_AccRange_t range,
                           uint8_t odr, uint8_t bwp) {

    // Set range
    if (!BMI088_AccelWriteReg(dev, BMI088_ACC_RANGE, range)) {
        return false;
    }

    // Set ODR and bandwidth
    uint8_t conf_val = (bwp << 4) | odr;
    if (!BMI088_AccelWriteReg(dev, BMI088_ACC_CONF, conf_val)) {
        return false;
    }

    dev->acc_range = range;
    dev->acc_sensitivity = BMI088_GetAccelSensitivity(range);

    return true;
}

/**
 * @brief Configure gyroscope
 */
bool BMI088_ConfigureGyro(BMI088_Handle_t *dev, BMI088_GyroRange_t range,
                          uint8_t bandwidth) {

    // Set range
    if (!BMI088_GyroWriteReg(dev, BMI088_GYRO_RANGE, range)) {
        return false;
    }

    // Set bandwidth
    if (!BMI088_GyroWriteReg(dev, BMI088_GYRO_BANDWIDTH, bandwidth)) {
        return false;
    }

    dev->gyro_range = range;
    dev->gyro_sensitivity = BMI088_GetGyroSensitivity(range);

    return true;
}

/**
 * @brief Read raw accelerometer data
 */
bool BMI088_ReadAccelRaw(BMI088_Handle_t *dev, BMI088_RawData_t *data) {
    uint8_t buffer[6];

    if (!BMI088_AccelReadReg(dev, BMI088_ACC_X_LSB, buffer, 6)) {
        return false;
    }

    data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return true;
}

/**
 * @brief Read raw gyroscope data
 */
bool BMI088_ReadGyroRaw(BMI088_Handle_t *dev, BMI088_RawData_t *data) {
    uint8_t buffer[6];

    if (!BMI088_GyroReadReg(dev, BMI088_GYRO_RATE_X_LSB, buffer, 6)) {
        return false;
    }

    data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return true;
}

/**
 * @brief Read accelerometer data in g
 */
bool BMI088_ReadAccel(BMI088_Handle_t *dev, BMI088_AccelData_t *data) {
    BMI088_RawData_t raw;

    if (!BMI088_ReadAccelRaw(dev, &raw)) {
        return false;
    }

    data->x = (float)raw.x / dev->acc_sensitivity;
    data->y = (float)raw.y / dev->acc_sensitivity;
    data->z = (float)raw.z / dev->acc_sensitivity;

    return true;
}

/**
 * @brief Read gyroscope data in °/s
 */
bool BMI088_ReadGyro(BMI088_Handle_t *dev, BMI088_GyroData_t *data) {
    BMI088_RawData_t raw;

    if (!BMI088_ReadGyroRaw(dev, &raw)) {
        return false;
    }

    data->x = (float)raw.x / dev->gyro_sensitivity;
    data->y = (float)raw.y / dev->gyro_sensitivity;
    data->z = (float)raw.z / dev->gyro_sensitivity;

    return true;
}

/**
 * @brief Read temperature in °C
 */
bool BMI088_ReadTemperature(BMI088_Handle_t *dev, float *temp) {
    uint8_t buffer[2];

    if (!BMI088_AccelReadReg(dev, BMI088_ACC_TEMP_MSB, buffer, 2)) {
        return false;
    }

    // Combine MSB and LSB (11-bit signed value)
    int16_t temp_raw = (int16_t)((buffer[0] << 3) | (buffer[1] >> 5));

    // Sign extend if negative (bit 10 is sign bit)
    if (temp_raw & 0x0400) {
        temp_raw |= 0xF800;
    }

    // Convert to temperature: temp = raw * 0.125 + 23
    *temp = (float)temp_raw * 0.125f + 23.0f;

    return true;
}

/**
 * @brief Soft reset both sensors
 */
bool BMI088_SoftReset(BMI088_Handle_t *dev) {
    if (!BMI088_AccelWriteReg(dev, BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET_CMD)) {
        return false;
    }
    HAL_Delay(50);

    if (!BMI088_GyroWriteReg(dev, BMI088_GYRO_SOFTRESET, BMI088_SOFT_RESET_CMD)) {
        return false;
    }
    HAL_Delay(50);

    return true;
}
