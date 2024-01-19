#include <Arduino.h>

int pwm[] = {12, 14, 26, 27};

void setup()
{
  Serial.begin(115200);
  for (int i = 0; i < 4; i++)
  {
    pinMode(pwm[i], OUTPUT);
  }
}

//173 - 128 = 45
//128 - 45 = 83

void forword()
{
  analogWrite(pwm[0], 83);
  analogWrite(pwm[1], 83);
  analogWrite(pwm[2], 173);
  analogWrite(pwm[3], 173);
}

void backword()
{
  digitalWrite(pwm[0], LOW);
  digitalWrite(pwm[1], HIGH);
  digitalWrite(pwm[2], LOW);
  digitalWrite(pwm[3], HIGH);
}

void left()
{
  digitalWrite(pwm[0], LOW);
  digitalWrite(pwm[1], HIGH);
  digitalWrite(pwm[2], HIGH);
  digitalWrite(pwm[3], LOW);
}

void right()
{
  digitalWrite(pwm[0], HIGH);
  digitalWrite(pwm[1], LOW);
  digitalWrite(pwm[2], LOW);
  digitalWrite(pwm[3], HIGH);
}

void loop()
{
  forword();
  delay(1000);
}