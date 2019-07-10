#include "Temprature.h"
#include <Arduino.h>

int Temprature::readTemp(){
    sensors->requestTemperatures(); 
    Serial.println(sensors->getTempCByIndex(0));
}