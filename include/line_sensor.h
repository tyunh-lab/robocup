#include <Arduino.h>
#include <pins.h>

void setupLineSensor()
{
    pinMode(FRONT_LINE_SENSOR_PIN, INPUT);
    pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
    pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
    pinMode(BACK_LINE_SENSOR_PIN, INPUT);
}

int readLineSensor(int pin)
{
    return analogRead(pin);
}