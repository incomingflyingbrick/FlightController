#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>

class Temprature
{
  public:
    OneWire* oneWire; 
    DallasTemperature* sensors;
    Temprature(int sclPin){
        Serial.begin(9600);
        Serial.println("1111111");
        oneWire = new OneWire(sclPin);
        sensors = new DallasTemperature(oneWire);
        sensors->begin();

    }
    int readTemp();
    
};
