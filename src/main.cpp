#include <Arduino.h>
#include "jyro.h"
#include "ble.cpp"
#include "cam.cpp"

// モーター設定
int pwm[] = {12, 14, 26, 27};

double angle;

void forword()
{
  analogWrite(pwm[0], 10);
  analogWrite(pwm[1], 236);
  analogWrite(pwm[2], 236);
  analogWrite(pwm[3], 10);
}

void back()
{
  analogWrite(pwm[0], 236);
  analogWrite(pwm[1], 10);
  analogWrite(pwm[2], 10);
  analogWrite(pwm[3], 236);
}

void left()
{
  analogWrite(pwm[0], 226);
  analogWrite(pwm[1], 226);
  analogWrite(pwm[2], 226);
  analogWrite(pwm[3], 226);
}

void right()
{
  analogWrite(pwm[0], 10);
  analogWrite(pwm[1], 10);
  analogWrite(pwm[2], 10);
  analogWrite(pwm[3], 10);
}

void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < 4; i++)
  {
    pinMode(pwm[i], OUTPUT);
  }

  // Lちか
  pinMode(2, OUTPUT);
  initBLE();
  initCAM();

  // Serial.setDebugOutput(true);
  // Serial.println();

  Serial.print("jyro setupping....");
  Wire.setPins(21, 22);
  jy901.StartIIC();
  Serial.println("done");
}

/*
explain:
モーターの周波数を設定する関数
usage:
  motor: motor pin 0-3 順に[12,14,26,27]
  frequency: 周波数 0-255
  useAutoConversion (optional): モーターの位置を考慮して、自動的に255を引くかどうか（デフォルトはfalse）
*/
void set_frequency_for_uart(int motor, int frequency, bool useAutoConversion = false)
{
  if (!useAutoConversion)
  {
    analogWrite(pwm[motor], frequency);
  }
  else
  {
    if (motor == 0 || motor == 3)
    {
      analogWrite(pwm[motor], 255 - frequency);
    }
    else
    {
      analogWrite(pwm[motor], frequency);
    }
  }
}

// 　TODO: Setup関数内で初期値を設定させるので、電源投下時を0として計算させる。
// 　      もしくは、回路を変更させずに絶対値にするのもあり。
double errorRange = 2; // 2度の誤差を許容する

void loop()
{
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  // このブロックはisolateさせて、真っ直ぐにさせる精度を上げるかも
  jy901.GetAngle();
  angle = (float)jy901.stcAngle.Angle[2] / 32768 * 180;
  /*ここからいけるかわからん*/
  pCharacteristic->setValue(angle);
  pCharacteristic->notify();
  /*ここまで*/
  Serial.print("angle:");
  Serial.println(angle);

  if (angle > errorRange)
  {
    while (angle > errorRange)
    {
      right();
    }

    Serial.println("right");
  }
  else if (angle < errorRange * -1)
  {
    while (angle < errorRange * -1)
    {
      left();
    }
    Serial.println("left");
  }
  else
  {
    forword();
    Serial.println("forword");
  }

  // delay(400);
  // digitalWrite(2, HIGH);
  // delay(400);
  // digitalWrite(2, LOW);
}
