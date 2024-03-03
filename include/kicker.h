#include <Arduino.h>
#include <pins.h>

void initKicker()
{
    pinMode(KICKER_PIN, OUTPUT);
}

void kick()
{
    digitalWrite(KICKER_PIN, HIGH);
    delay(100);
    digitalWrite(KICKER_PIN, LOW);
}