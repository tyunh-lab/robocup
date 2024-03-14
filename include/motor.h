#include <Arduino.h>
#include <manager.h>
#include <pins.h>
#include <PID_v1.h>

#ifndef ROBOCUP_MOTOR
#define ROBOCUP_MOTOR

int motor_pins[4] = {MOTOR1_PIN1, MOTOR1_PIN2, MOTOR2_PIN1, MOTOR2_PIN2};

int motor_power_index[4] = {1, 1, 1, -1};

int forward[4] = {56, 200, 56, 200};
int left[4] = {200, 200, 56, 56};
int backward[4] = {200, 56, 200, 56};
int right[4] = {56, 56, 200, 200};

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
    Setpoint = 0;
    Input = angle;
    pid_roll.Compute();
    Serial.println(Output);
    for (int i = 0; i < 4; i++)
    {
        analogWrite(motor_pins[i], 128 - Output);
    }
}

// // PID制御のパラメータ
// float Kp = 1.0;
// float Ki = 0.0;
// float Kd = 0.0;

// // PID制御器の初期値
// float integral = 0;
// float prev_error = 0;

// // 目標値 (0度)
// float target_angle = 0;

// // 制御周期 (ミリ秒) <-いるかわからん
// unsigned long dt = 100;

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

// float pidControl(float currentAngle)
// {
//     float error = target_angle - currentAngle;
//     integral = integral + error * dt / 1000.0;
//     float derivative = (error - prev_error) / (dt / 1000.0);
//     float output = Kp * error + Ki * integral + Kd * derivative;
//     prev_error = error;
//     delay(dt);
//     return output;
// }

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
        analogWrite(motor_pins[i], 128);
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
    if (now_angle > 10)
    {
        for (int i = 0; i < 4; i++)
        {
            if (i % 2 == 1)
            {
                set_motor(i, addSpeed(forward[i], 35));
                analogWrite(motor_pins[i], addSpeed(forward[i], 35));
            }
            else
            {
                set_motor(i, forward[i]);
                analogWrite(motor_pins[i], forward[i]);
            }
        }
    }
    else if (now_angle < -10)
    {
        for (int i = 0; i < 4; i++)
        {
            if (i % 2 == 0)
            {
                set_motor(i, addSpeed(forward[i], 35));
                analogWrite(motor_pins[i], addSpeed(forward[i], 35));
            }
            else
            {
                set_motor(i, forward[i]);
                analogWrite(motor_pins[i], forward[i]);
            }
        }
    }
    else
    {
        set_motor(0, forward[0]);
        set_motor(1, forward[1]);
        set_motor(2, forward[2]);
        set_motor(3, forward[3]);
        moveForward();
    }
}

void test_motor(int index, bool all = false)
{
    if (index != -1)
    {

        stop();
        analogWrite(motor_pins[index], 254);
    }
    if (all)
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