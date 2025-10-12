/* ublox_m8c.h - UBLOX M8C GNSS Module Driver */

#ifndef UBLOX_M8C_H
#define UBLOX_M8C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define NMEA_BUFFER_SIZE    256
#define NMEA_MAX_SENTENCES  10

/* NMEA Sentence Types */
typedef enum {
    NMEA_GPGGA,  // GPS Fix Data
    NMEA_GPGLL,  // Geographic Position
    NMEA_GPGSA,  // GPS DOP and active satellites
    NMEA_GPGSV,  // GPS Satellites in view
    NMEA_GPRMC,  // Recommended Minimum Navigation Information
    NMEA_GPVTG,  // Track made good and ground speed
    NMEA_UNKNOWN
} NMEA_Type_t;

/* GPS Fix Quality */
typedef enum {
    GPS_NO_FIX = 0,
    GPS_FIX = 1,
    GPS_DGPS_FIX = 2
} GPS_FixQuality_t;

/* GPS Data Structure */
typedef struct {
    float latitude;       // Decimal degrees
    char lat_dir;         // N or S
    float longitude;      // Decimal degrees
    char lon_dir;         // E or W
    float altitude;       // Meters above sea level
    uint8_t satellites;   // Number of satellites in use
    GPS_FixQuality_t fix_quality;
    float hdop;           // Horizontal dilution of precision
    float speed_knots;    // Speed over ground in knots
    float course;         // Track angle in degrees
    bool valid;           // Data valid flag
} GPS_Data_t;

/* UBLOX M8C Handle */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[NMEA_BUFFER_SIZE];
    char nmea_sentence[NMEA_BUFFER_SIZE];
    uint16_t nmea_index;
    bool sentence_ready;
    GPS_Data_t gps_data;
} UBLOX_M8C_Handle_t;

/* Function Prototypes */
void UBLOX_M8C_Init(UBLOX_M8C_Handle_t *dev, UART_HandleTypeDef *huart);
void UBLOX_M8C_ProcessByte(UBLOX_M8C_Handle_t *dev, uint8_t byte);
bool UBLOX_M8C_ParseNMEA(UBLOX_M8C_Handle_t *dev);
NMEA_Type_t UBLOX_M8C_GetSentenceType(const char *sentence);
bool UBLOX_M8C_ParseGPGGA(UBLOX_M8C_Handle_t *dev, const char *sentence);
bool UBLOX_M8C_ParseGPRMC(UBLOX_M8C_Handle_t *dev, const char *sentence);
float UBLOX_M8C_ConvertToDecimalDegrees(const char *coord, char direction);
bool UBLOX_M8C_HasNewData(UBLOX_M8C_Handle_t *dev);
void UBLOX_M8C_GetData(UBLOX_M8C_Handle_t *dev, GPS_Data_t *data);
const char* UBLOX_M8C_GetLastSentence(UBLOX_M8C_Handle_t *dev);

#endif /* UBLOX_M8C_H */
