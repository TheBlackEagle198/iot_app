#include "TemperatureModule.h"

HumidityTemperatureModule module(
    0x0,       // defaultGlobalId
    7,         // cePin
    8,         // csPin
    2,         // connectButtonPin
    3,         // sensorPin
    4,         // statusLedPin
    1.0f,      // temperatureThreshold
    1.0f);     // humidityThreshold

/// main code ******************************************************************
void setup()
{
    Serial.begin(115200);
    module.init();
    Serial.println("Module initialized");
}

void loop()
{
    module.run();
}