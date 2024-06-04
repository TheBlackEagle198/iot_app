#include "TemperatureModule.h"

HumidityTemperatureModule module(
    0x0, 
    7, 
    8, 
    2,
    3,
    1.0f,
    1.0f);

/// main code ******************************************************************
void setup()
{
    Serial.begin(115200);
    module.init();
}

void loop()
{
    module.run();
}