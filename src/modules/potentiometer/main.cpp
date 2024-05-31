#include "PotentiometerModule.h"

PotentiometerModule module(
    0x0,
    7, // ce pin
    8, // cs pin
    2, // connect button pin
    A0,// potentiometer pin
    100); // threshold

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