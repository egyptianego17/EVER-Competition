#include <Arduino.h>
#include "../lib/STD_TYPES.hpp"
#include "../lib/LabsCounter.hpp"

void setup()
{
  Serial.begin(9600); 
  if (initGPS() == STD_TYPES_OK)
  {
    Serial.println("GPS initialized successfully!");
  }
  else
  {
    Serial.println("GPS initialization failed!");
  }
  if (setStartPoint() == STD_TYPES_OK)
  {
    Serial.println("Start point set successfully!");
  }
  else
  {
    Serial.println("Start point setting failed!");
  }
}

void loop()
{
  Serial.println("Laps count: " + String(getLapsCount()));
}

