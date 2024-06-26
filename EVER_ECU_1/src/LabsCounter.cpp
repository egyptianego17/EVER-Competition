#include <TimerOne.h>
#include <TinyGPS.h>
#include "../lib/STD_TYPES.hpp"
#include "../lib/LabsCounter.hpp"
#include <SoftwareSerial.h>
#include <math.h>

/* Constants */
const float R = 6371000; /* Radius of the Earth in meters */

/* Function prototypes */
static float haversineDistance(float lat1, float lon1, float lat2, float lon2);
uint8_t initGPS(void);
uint8_t setStartPoint(void);
uint8_t getLocation(float *lat, float *lon);
static uint8_t checkNewLAP(void);

/* Global variables */
TinyGPS gps;
static float latest_lat, latest_lon;
static float startLat, startLon;
static uint8_t labCounter = 0;
const int interval = 1000000;  /* Interval in microseconds (1 second = 1000000 microseconds) */

uint8_t initGPS(void)
{
  uint8_t ERROR_STATUS = STD_TYPES_OK;

  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("GPS setup...");
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  /* For one second we parse GPS data and report some key values */
  for (unsigned long start = millis(); millis() - start < GPS_TIMEOUT;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      if (gps.encode(c)) /* Did a new valid sentence come in? */
        newData = true;
    }
  }

  gps.stats(&chars, &sentences, &failed);
  if (chars == 0 || failed == 1 || !newData)
  {
    Serial.println("Init GPS failed, check the wiring!");
    ERROR_STATUS = STD_TYPES_NOK;
  }
  Serial.println("GPS setup done!");
  return ERROR_STATUS;
}

uint8_t setStartPoint(void)
{
  float lat, lon;
  Serial.println("Setting start point...");
  uint8_t ERROR_STATUS = getLocation(&lat, &lon);
  if (ERROR_STATUS == STD_TYPES_OK)
  {
    startLat = lat;
    startLon = lon;
    Serial.println("Start point set successfully.");
    Serial.println("Start point: LAT=" + String(startLat) + ", LON=" + String(startLon));
    Timer1.initialize(interval);  /* Initialize Timer1 with the interval */
    Timer1.attachInterrupt(checkNewLAP);  /* Attach the interrupt handler function */
  }
  else
  {
    Serial.println("Failed to set start point.");
  }
  return ERROR_STATUS;
}

uint8_t getLocation(float *lat, float *lon)
{
  uint8_t ERROR_STATUS = STD_TYPES_OK;
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  /* For one second we parse GPS data and report some key values */
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      if (gps.encode(c)) /* Did a new valid sentence come in? */
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    latest_lat = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    latest_lon = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
    *lat = latest_lat;
    *lon = latest_lon;
    Serial.println("LAT= " + String(latest_lat) + ", LON= " + String(latest_lon));
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.println(" CHARS= " + String(chars) + ", SENTENCES= " + String(sentences) + ", CSUM ERR= " + String(failed));

  if (failed == 1)
  {
    Serial.println("Reading GPS failed, check the wiring!");
    ERROR_STATUS = STD_TYPES_NOK;
  }
  return ERROR_STATUS;
}

uint8_t getLapsCount(void)
{
  return labCounter;
}

static uint8_t checkNewLAP(void)
{
  uint8_t ERROR_STATUS = STD_TYPES_NOK;
  static unsigned long lastLapTime = millis();
  if (millis() - lastLapTime < 30000) /* 30 second */
  {
    return STD_TYPES_NOK;
  }
  else
  {
    float lat, lon;
    ERROR_STATUS = getLocation(&lat, &lon);
    if (ERROR_STATUS == STD_TYPES_OK && !isnan(startLat) && !isnan(startLon))
    {
      float distance = haversineDistance(startLat, startLon, lat, lon);
      if (distance <= TRACK_WIDTH) /* 10 meters radius */
      {
        labCounter++;
        lastLapTime = millis();
        Serial.println("New lap detected!");
        Serial.println("Lap count: " + String(labCounter));
      }
    }
    else
    {
      Serial.println("Failed to get location.");
    }
  }

  return ERROR_STATUS;
}

static float haversineDistance(float lat1, float lon1, float lat2, float lon2)
{
  /* Convert degrees to radians */
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);

  /* Haversine formula */
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1) * cos(lat2) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = R * c;

  return distance; /* Distance in meters */
}

