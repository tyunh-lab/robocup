#include <Arduino.h>
#include <pins.h>

int motor_pins[4] = {MOTOR1_PIN1, MOTOR1_PIN2, MOTOR2_PIN1, MOTOR2_PIN2};

void initMotor()
{
    for (int i = 0; i < 4; i++)
    {
        ledcSetup(i, 12800, 8);
        ledcAttachPin(motor_pins[i], i);
    }
}

void moveForward()
{
    ledcWrite(0, 255);
    ledcWrite(1, 0);
    ledcWrite(2, 255);
    ledcWrite(3, 0);
}

void moveBackward()
{
    ledcWrite(0, 0);
    ledcWrite(1, 255);
    ledcWrite(2, 0);
    ledcWrite(3, 255);
}

void moveLeft()
{
    ledcWrite(0, 0);
    ledcWrite(1, 255);
    ledcWrite(2, 255);
    ledcWrite(3, 0);
}

void moveRight()
{
    ledcWrite(0, 255);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 255);
}