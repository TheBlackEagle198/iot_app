#include "BooleanModule.h"

BooleanModule module(0x0, 7, 8, 2, 4);

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