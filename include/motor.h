#include <Arduino.h>
#include <manager.h>
#include <pins.h>
#include <PID_v1.h>
#include <jyro.h>

#ifndef ROBOCUP_MOTOR
#define ROBOCUP_MOTOR

int motor_pins[4] = {MOTOR1_PIN1, MOTOR1_PIN2, MOTOR2_PIN1, MOTOR2_PIN2};

int forward[4] = {56, 200, 56, 200};
int left[4] = {180, 180, 86, 86};
int backward[4] = {200, 76, 200, 76};
int right[4] = {86, 86, 180, 180};

int left_rotate[4] = {200, 200, 200, 200};
int right_rotate[4] = {56, 56, 56, 56};

int left_forward[4] = {126, 180, 76, 126};
int left_backward[4] = {180, 126, 126, 76};

int right_forward[4] = {76, 126, 126, 180};
int right_backward[4] = {126, 76, 180, 126};

// PIDライブラリ用
double Setpoint, Input, Output;

double Kp = 1, Ki = 0, Kd = 0.0005;
PID pid_roll(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void pid_setup()
{
    pid_roll.SetMode(AUTOMATIC);
    pid_roll.SetSampleTime(5);
    pid_roll.SetOutputLimits(-125, 125);
}

void pid_controll_motor(double angle)
{
    Input = angle;
    Setpoint = 0;
    pid_roll.Compute();
    Serial.println(Output);
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], 128 - Output);
    }
}

int addSpeed(int original_speed, int add_speed)
{
    if (original_speed >= 128)
    {
        return min(original_speed + add_speed, 250);
    }
    else if (original_speed < 128)
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
    analogWriteResolution(8);     // 8bitの分解能
    analogWriteFrequency(169000); // 100kHzのPWM周波数
}
void moveForward()
{
    // Setpoint = 0;
    // Input = get_angle_with_heading();
    // pid_roll.Compute();
    // for (int i = 0; i < 4; i++)
    // {
    //     if (i % 2 == 0)
    //     {
    //         analogWrite(motor_pins[i], addSpeed(forward[i], -Output));
    //     }
    //     else
    //     {
    //         analogWrite(motor_pins[i], addSpeed(forward[i], Output));
    //     }
    // }
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], forward[i]);
    }
}

void moveLeft()
{
    // Setpoint = 0;
    // Input = get_angle_with_heading();
    // pid_roll.Compute();
    // for (int i = 0; i < 4; i++)
    // {
    //     if (i % 2 == 0)
    //     {
    //         analogWrite(motor_pins[i], addSpeed(left[i], -Output));
    //     }
    //     else
    //     {
    //         analogWrite(motor_pins[i], addSpeed(left[i], Output));
    //     }
    // }

    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], left[i]);
    }
}
void moveBackward()
{
    // Setpoint = 0;
    // Input = get_angle_with_heading();
    // pid_roll.Compute();
    // for (int i = 0; i < 4; i++)
    // {
    //     if (i % 2 == 0)
    //     {
    //         analogWrite(motor_pins[i], addSpeed(backward[i], -Output));
    //     }
    //     else
    //     {
    //         analogWrite(motor_pins[i], addSpeed(backward[i], Output));
    //     }
    // }
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], backward[i]);
    }
}
void moveRight()
{
    // Input = get_angle_with_heading();
    // Setpoint = 0;
    // pid_roll.Compute();
    // for (int i = 0; i < 4; i++)
    // {
    //     if (i % 2 == 0)
    //     {
    //         analogWrite(motor_pins[i], addSpeed(right[i], -Output));
    //     }
    //     else
    //     {
    //         analogWrite(motor_pins[i], addSpeed(right[i], Output));
    //     }
    // }
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
        analogWrite(motor_pins[i], 128);
    }
}

void moveWith_angleCorrection(double angle)
{
    Input = angle;
    pid_roll.Compute();
    Serial.println(Output);
    for (int i = 0; i < 4; i++)
    {
        if (i % 2 == 0)
        {
            analogWrite(motor_pins[i], addSpeed(forward[i], -Output));
        }
        else
        {
            analogWrite(motor_pins[i], addSpeed(forward[i], Output));
        }
    }
}

void face_forward(double angle, double target_angle = 0)
{
    Serial.println("current:" + String(angle) + "°");
    Serial.println("target:" + String(target_angle) + "°");
    Serial.println("calced:" + String(angle + target_angle) + "°");
    Input = angle;
    Setpoint = 0;
    while (abs(Input - Setpoint) > 5)
    {
        pid_roll.Compute();
        Serial.print("target:" + String(Setpoint) + "\t");
        Serial.print("current:" + String(angle) + "\t");
        Serial.println("Output" + String(Output));
        for (int i = 0; i < 4; i++)
        {
            if (i % 2 == 0)
            {
                analogWrite(motor_pins[i], addSpeed(forward[i], -Output));
            }
            else
            {
                analogWrite(motor_pins[i], addSpeed(forward[i], Output));
            }
        }
        angle = get_angle_with_heading();
        Input = angle;
    }
    stop();
}

void test_motor(int index)
{
    if (index != -1)
    {
        stop();
        analogWrite(motor_pins[index], 254);
        delay(500);
    }
    else
    {
        moveForward();
        delay(3000);
        moveLeft();
        delay(3000);
        moveBackward();
        delay(3000);
        moveRight();
        delay(3000);
        stop();
        delay(3000);

        rotateLeft(200);
        delay(1000);
        stop();
        delay(1000);
        rotateRight(200);
        delay(1000);
        stop();
        delay(1000);
    }
}

#endif