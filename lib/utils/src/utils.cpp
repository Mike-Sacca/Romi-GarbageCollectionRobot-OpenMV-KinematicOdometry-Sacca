#include "utils.h"

void TeleplotPrint(const char* var, float value)
{
    Serial.print('>');
    Serial.print(var);
    Serial.print(':');
    Serial.print(value);
    Serial.print('\n');
}

void TeleplotPrintXY(const char* topic, float x, float y)
{
    SerialUSB.print("\n>");
    SerialUSB.print(topic);
    SerialUSB.print(':');
    SerialUSB.print(x, 5);
    SerialUSB.print(':');
    SerialUSB.print(y, 5);
    SerialUSB.println("|xy");
}