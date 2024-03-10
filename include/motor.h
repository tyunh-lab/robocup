#include <Arduino.h>
#include <pins.h>

int motor_pins[4] = {MOTOR1_PIN1, MOTOR1_PIN2, MOTOR2_PIN1, MOTOR2_PIN2};

int forward[4] = {56, 200, 56, 56};
int left[4] = {200, 200, 56, 200};
int backward[4] = {200, 56, 200, 200};
int right[4] = {56, 56, 200, 56};

int left_rotate[4] = {200, 200, 200, 56};
int right_rotate[4] = {56, 56, 56, 200};

int left_forward[4] = {126, 180, 76, 126};
int left_backward[4] = {180, 126, 126, 180};

int right_forward[4] = {76, 126, 126, 76};
int right_backward[4] = {126, 76, 180, 126};

int addSpeed(int original_speed, int add_speed)
{
    if (original_speed >= 127)
    {
        return min(original_speed + add_speed, 250);
    }
    else if (original_speed <= 126)
    {
        return max(original_speed - add_speed, 5);
    }
    else
    {
        return original_speed;
    }
}

void initMotor()
{
    for (int i = 0; i < 4; i++)
    {
        pinMode(motor_pins[i], OUTPUT);
    }
    analogWriteFrequency(500000); // 100kHz
}
void moveForward()
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], forward[i]);
    }
}
void moveLeft()
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], left[i]);
    }
}
void moveBackward()
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], backward[i]);
    }
}
void moveRight()
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], right[i]);
    }
}
void rotateLeft(int speed = 0)
{
    for (int i = 0; i < 4; i++)
    {
        int added_speed = addSpeed(left_rotate[i], speed);
        analogWrite(motor_pins[i], added_speed);
    }
}
void rotateRight(int speed = 0)
{
    for (int i = 0; i < 4; i++)
    {
        int added_speed = addSpeed(right_rotate[i], speed);
        analogWrite(motor_pins[i], added_speed);
    }
}
void stop()
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], 125);
    }
}
void moveLeftForward()
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], left_forward[i]);
    }
}
void moveLeftBackward()
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], left_backward[i]);
    }
}
void moveRightForward()
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], right_forward[i]);
    }
}
void moveRightBackward()
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], right_backward[i]);
    }
}

void moveWith_angleCorrection(double now_angle)
{
    if (now_angle > 5)
    {
        for (int i = 0; i < 4; i++)
        {
            analogWrite(motor_pins[i], addSpeed(forward[i], left_rotate[i]));
        }
    }
    else if (now_angle < -5)
    {
        for (int i = 0; i < 4; i++)
        {
            analogWrite(motor_pins[i], addSpeed(forward[i], right_rotate[i]));
        }
    }
    else
    {
        moveForward();
    }
}