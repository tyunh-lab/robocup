#include <Arduino.h>
#include <pins.h>

int line_sensor_pins[4] = {FRONT_LINE_SENSOR_PIN, LEFT_LINE_SENSOR_PIN, RIGHT_LINE_SENSOR_PIN, BACK_LINE_SENSOR_PIN};

void setupLineSensor()
{
    pinMode(FRONT_LINE_SENSOR_PIN, INPUT);
    pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
    pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
    pinMode(BACK_LINE_SENSOR_PIN, INPUT);
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