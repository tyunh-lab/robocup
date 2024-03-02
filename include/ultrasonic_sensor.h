#include <Arduino.h>
#include <pins.h>

double duration = 0;
double distance = 0;
double speed_of_sound = 331.5 + 0.6 * 25; // 25℃の気温の想定

void setupUltrasonicSensor()
{
    pinMode(FRONT_TRIG, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);
    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);
    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);
    pinMode(BACK_TRIG, OUTPUT);
    pinMode(BACK_ECHO, INPUT);
}

/*
index 0: front
index 1: left
index 2: right
index 3: back
*/
double readUltrasonicSensor(int index)
{
    int trig, echo;

    switch (index)
    {
    case 0:
        trig = FRONT_TRIG;
        echo = FRONT_ECHO;
        break;
    case 1:
        trig = LEFT_TRIG;
        echo = LEFT_ECHO;
        break;
    case 2:
        trig = RIGHT_TRIG;
        echo = RIGHT_ECHO;
        break;
    case 3:
        trig = BACK_TRIG;
        echo = BACK_ECHO;
        break;

    default:
        Serial.println("Invalid index");
        break;
    }

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