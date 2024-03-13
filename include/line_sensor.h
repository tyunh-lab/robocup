#include <Arduino.h>
#include <pins.h>

#ifndef ROBOCUP_LINE_SENSOR
#define ROBOCUP_LINE_SENSOR

int line_sensor_pins[4] = {FRONT_LINE_SENSOR_PIN, LEFT_LINE_SENSOR_PIN, RIGHT_LINE_SENSOR_PIN, BACK_LINE_SENSOR_PIN};

void setupLineSensor()
{
    for (int i = 0; i < 4; i++)
    {
        pinMode(line_sensor_pins[i], INPUT);
    }
}

/*
index 0: front
index 1: left
index 2: right
index 3: back

This function return the line sensors data.
*/
int readLineSensor(int index)
{
    return analogRead(line_sensor_pins[index]);
}

#endif