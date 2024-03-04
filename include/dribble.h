#include <Arduino.h>
#include <Servo.h>
#include <pins.h>

Servo esc; // Servoオブジェクトを作成

void initDribbler()
{
    esc.attach(DRIBBLER_MOTOR_PIN); // サーボモータをDribblerピンに接続
}

void dribble()
{
    // 停止
    esc.writeMicroseconds(1000);
    delay(2000);
    // ハーフスロットル
    esc.writeMicroseconds(1500);
    delay(2000);
    // フルスロットル
    esc.writeMicroseconds(2000);
    delay(2000);
}