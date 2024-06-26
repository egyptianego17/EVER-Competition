#ifndef _LABSCOUNTER_HPP
#define _LABSCOUNTER_HPP

/* This flag is used to control serial hardware or software 
    0 -> Hardware
    1 -> Software                                         */
#define SW_SERIAL_FLAG 0

/* Those macros are used to define the SW TX and RX in case you are using software serial and ignored in Hardware */
#define SW_GPS_RX 1
#define SW_GPS_TX 2
#define TRACK_WIDTH 20.0
#define GPS_TIMEOUT 3000

static float haversineDistance(float lat1, float lon1, float lat2, float lon2);
uint8_t initGPS(void);
uint8_t setStartPoint(void);
uint8_t getLocation(float *lat, float *lon);
static uint8_t checkNewLAP(void);
uint8_t getLapsCount(void);

#endif