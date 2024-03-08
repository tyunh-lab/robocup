// #include <Arduino.h>
#include <Servo.h>
#include <pins.h>

Servo esc; // Servoオブジェクトを作成

void initDribbler()
{
    esc.attach(DRIBBLER_MOTOR_PIN); // サーボモータをDribblerピンに接続
    esc.writeMicroseconds(1000);    // サーボモータを初期位置に
}

// // 停止
// esc.writeMicroseconds(1000);
// delay(2000);

void dribble()
{
    // ハーフスロットル
    esc.writeMicroseconds(1500);
}

void power_dribble()
{
    // フルスロットル
    esc.writeMicroseconds(2000);
}

void stop_dribble()
{
    esc.writeMicroseconds(1000);
}