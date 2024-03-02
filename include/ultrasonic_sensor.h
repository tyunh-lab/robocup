#include <Arduino.h>
#include <pins.h>

int ultrasonic_sensor_pins[4][2] = {
    {FRONT_TRIG, FRONT_ECHO},
    {LEFT_TRIG, LEFT_ECHO},
    {RIGHT_TRIG, RIGHT_ECHO},
    {BACK_TRIG, BACK_ECHO}};

double duration = 0;
double distance = 0;
double speed_of_sound = 331.5 + 0.6 * 25; // 25℃の気温の想定

void setupUltrasonicSensor()
{
    for (int i = 0; i < 4; i++)
    {
        pinMode(ultrasonic_sensor_pins[i][0], OUTPUT);
        pinMode(ultrasonic_sensor_pins[i][1], INPUT);
    }
}

/*
index 0: front
index 1: left
index 2: right
index 3: back

This function return the distance.
Unit is cm.
*/
double readUltrasonicSensor(int index)
{
    int trig = ultrasonic_sensor_pins[index][0];
    int echo = ultrasonic_sensor_pins[index][1];

    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH); // 往復にかかった時間が返却される[マイクロ秒]

    if (duration > 0)
    {
        duration = duration / 2; // 往路にかかった時間
        distance = duration * speed_of_sound * 100 / 1000000;
    }
    return distance;
}