#include <Arduino.h>
#include <pins.h>

#ifndef ROBOCUP_KICKER
#define ROBOCUP_KICKER

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

#endif