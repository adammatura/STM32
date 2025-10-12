/* ublox_m8c.c - UBLOX M8C GNSS Module Driver Implementation */

#include "ublox_m8c.h"
#include <stdlib.h>
#include <stdio.h>

/**
 * @brief Initialize UBLOX M8C module
 */
void UBLOX_M8C_Init(UBLOX_M8C_Handle_t *dev, UART_HandleTypeDef *huart) {
    dev->huart = huart;
    dev->nmea_index = 0;
    dev->sentence_ready = false;
    memset(&dev->gps_data, 0, sizeof(GPS_Data_t));

    // Start UART reception in interrupt mode (single byte)
    HAL_UART_Receive_IT(dev->huart, dev->rx_buffer, 1);
}

/**
 * @brief Process incoming byte (call from UART interrupt)
 */
void UBLOX_M8C_ProcessByte(UBLOX_M8C_Handle_t *dev, uint8_t byte) {
    // Look for NMEA sentence start
    if (byte == '$') {
        dev->nmea_index = 0;
        dev->sentence_ready = false;
    }

    // Store byte
    if (dev->nmea_index < NMEA_BUFFER_SIZE - 1) {
        dev->nmea_sentence[dev->nmea_index++] = byte;

        // Check for sentence end (CR+LF)
        if (byte == '\n' && dev->nmea_index > 1 &&
            dev->nmea_sentence[dev->nmea_index - 2] == '\r') {
            dev->nmea_sentence[dev->nmea_index] = '\0';
            dev->sentence_ready = true;
        }
    }
}

/**
 * @brief Get sentence type
 */
NMEA_Type_t UBLOX_M8C_GetSentenceType(const char *sentence) {
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) return NMEA_GPGGA;
    if (strncmp(sentence, "$GPGLL", 6) == 0 || strncmp(sentence, "$GNGLL", 6) == 0) return NMEA_GPGLL;
    if (strncmp(sentence, "$GPGSA", 6) == 0 || strncmp(sentence, "$GNGSA", 6) == 0) return NMEA_GPGSA;
    if (strncmp(sentence, "$GPGSV", 6) == 0 || strncmp(sentence, "$GNGSV", 6) == 0) return NMEA_GPGSV;
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) return NMEA_GPRMC;
    if (strncmp(sentence, "$GPVTG", 6) == 0 || strncmp(sentence, "$GNVTG", 6) == 0) return NMEA_GPVTG;
    return NMEA_UNKNOWN;
}

/**
 * @brief Convert NMEA coordinate to decimal degrees
 */
float UBLOX_M8C_ConvertToDecimalDegrees(const char *coord, char direction) {
    if (coord == NULL || strlen(coord) < 4) return 0.0f;

    char degrees_str[4] = {0};
    float degrees, minutes;

    // Latitude: DDMM.MMMM, Longitude: DDDMM.MMMM
    int deg_len = (strlen(coord) > 10) ? 3 : 2;

    strncpy(degrees_str, coord, deg_len);
    degrees = atof(degrees_str);
    minutes = atof(coord + deg_len);

    float decimal = degrees + (minutes / 60.0f);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

/**
 * @brief Parse GPGGA sentence (GPS Fix Data)
 */
bool UBLOX_M8C_ParseGPGGA(UBLOX_M8C_Handle_t *dev, const char *sentence) {
    char *token;
    char sentence_copy[NMEA_BUFFER_SIZE];
    int field = 0;

    strncpy(sentence_copy, sentence, NMEA_BUFFER_SIZE - 1);
    token = strtok(sentence_copy, ",");

    while (token != NULL && field < 15) {
        switch (field) {
            case 2: // Latitude
                if (strlen(token) > 0) {
                    char lat_str[16];
                    strncpy(lat_str, token, 15);
                    token = strtok(NULL, ",");
                    field++;
                    if (token && strlen(token) > 0) {
                        dev->gps_data.lat_dir = token[0];
                        dev->gps_data.latitude = UBLOX_M8C_ConvertToDecimalDegrees(lat_str, dev->gps_data.lat_dir);
                    }
                }
                break;

            case 4: // Longitude
                if (strlen(token) > 0) {
                    char lon_str[16];
                    strncpy(lon_str, token, 15);
                    token = strtok(NULL, ",");
                    field++;
                    if (token && strlen(token) > 0) {
                        dev->gps_data.lon_dir = token[0];
                        dev->gps_data.longitude = UBLOX_M8C_ConvertToDecimalDegrees(lon_str, dev->gps_data.lon_dir);
                    }
                }
                break;

            case 6: // Fix quality
                if (strlen(token) > 0) {
                    dev->gps_data.fix_quality = (GPS_FixQuality_t)atoi(token);
                    dev->gps_data.valid = (dev->gps_data.fix_quality > GPS_NO_FIX);
                }
                break;

            case 7: // Number of satellites
                if (strlen(token) > 0) {
                    dev->gps_data.satellites = atoi(token);
                }
                break;

            case 8: // HDOP
                if (strlen(token) > 0) {
                    dev->gps_data.hdop = atof(token);
                }
                break;

            case 9: // Altitude
                if (strlen(token) > 0) {
                    dev->gps_data.altitude = atof(token);
                }
                break;
        }

        token = strtok(NULL, ",");
        field++;
    }

    return dev->gps_data.valid;
}

/**
 * @brief Parse GPRMC sentence (Recommended Minimum Navigation)
 */
bool UBLOX_M8C_ParseGPRMC(UBLOX_M8C_Handle_t *dev, const char *sentence) {
    char *token;
    char sentence_copy[NMEA_BUFFER_SIZE];
    int field = 0;

    strncpy(sentence_copy, sentence, NMEA_BUFFER_SIZE - 1);
    token = strtok(sentence_copy, ",");

    while (token != NULL && field < 12) {
        switch (field) {
            case 2: // Status (A=active, V=void)
                if (strlen(token) > 0) {
                    dev->gps_data.valid = (token[0] == 'A');
                }
                break;

            case 7: // Speed over ground (knots)
                if (strlen(token) > 0) {
                    dev->gps_data.speed_knots = atof(token);
                }
                break;

            case 8: // Track angle (degrees)
                if (strlen(token) > 0) {
                    dev->gps_data.course = atof(token);
                }
                break;
        }

        token = strtok(NULL, ",");
        field++;
    }

    return dev->gps_data.valid;
}

/**
 * @brief Parse NMEA sentence
 */
bool UBLOX_M8C_ParseNMEA(UBLOX_M8C_Handle_t *dev) {
    if (!dev->sentence_ready) {
        return false;
    }

    NMEA_Type_t type = UBLOX_M8C_GetSentenceType(dev->nmea_sentence);
    bool parsed = false;

    switch (type) {
        case NMEA_GPGGA:
            parsed = UBLOX_M8C_ParseGPGGA(dev, dev->nmea_sentence);
            break;

        case NMEA_GPRMC:
            parsed = UBLOX_M8C_ParseGPRMC(dev, dev->nmea_sentence);
            break;

        default:
            // Other sentence types not parsed yet
            break;
    }

    dev->sentence_ready = false;
    return parsed;
}

/**
 * @brief Check if new data is available
 */
bool UBLOX_M8C_HasNewData(UBLOX_M8C_Handle_t *dev) {
    return dev->sentence_ready;
}

/**
 * @brief Get GPS data
 */
void UBLOX_M8C_GetData(UBLOX_M8C_Handle_t *dev, GPS_Data_t *data) {
    memcpy(data, &dev->gps_data, sizeof(GPS_Data_t));
}

/**
 * @brief Get last received NMEA sentence
 */
const char* UBLOX_M8C_GetLastSentence(UBLOX_M8C_Handle_t *dev) {
    return dev->nmea_sentence;
}
