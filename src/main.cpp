// #include <Arduino.h>
// #include "jyro.h"
// // #include "ble.cpp"
// #include "cam.h"

// // モーター設定
// int pwm[] = {12, 14, 26, 27};

// double angle;

// void setup()
// {
//   Serial.begin(115200);

//   // モーター初期化
//   for (int i = 0; i < 4; i++)
//   {
//     pinMode(pwm[i], OUTPUT);
//     ledcSetup(i, 169000, 8); // 100kHz, 8bit(256段階)
//     ledcAttachPin(pwm[i], i);
//   }

//   // Lちか
//   pinMode(2, OUTPUT);
//   // initBLE();
//   initCAM();

//   // Serial.setDebugOutput(true);
//   // Serial.println();

//   Serial.print("jyro setupping....");
//   Wire.setPins(21, 22);
//   jy901.StartIIC();
//   Serial.println("done");
// }

// void stop()
// {
//   ledcWrite(0, 128);
//   ledcWrite(1, 120);
//   ledcWrite(2, 130);
//   ledcWrite(3, 118);
// }

// void foward()
// {
//   ledcWrite(0, 10);
//   ledcWrite(1, 200);
//   ledcWrite(2, 10);
//   ledcWrite(3, 200);
// }

// void back()
// {
//   ledcWrite(0, 200);
//   ledcWrite(1, 10);
//   ledcWrite(2, 200);
//   ledcWrite(3, 10);
// }

// void left()
// {
//   ledcWrite(0, 10);
//   ledcWrite(1, 10);
//   ledcWrite(2, 200);
//   ledcWrite(3, 200);
// }

// void right()
// {
//   ledcWrite(0, 200);
//   ledcWrite(1, 200);
//   ledcWrite(2, 10);
//   ledcWrite(3, 10);
// }

// /*
// explain:
// モーターの周波数を設定する関数
// usage:
//   motor: motor pin 0-3 順に[12,14,26,27]
//   frequency: 周波数 0-255
//   useAutoConversion (optional): モーターの位置を考慮して、自動的に255を引くかどうか（デフォルトはfalse）
// */
// void set_frequency_for_uart(int motor, int frequency, bool useAutoConversion = false)
// {
//   if (!useAutoConversion)
//   {
//     analogWrite(motor, frequency);
//   }
//   else
//   {
//     if (motor == 0 || motor == 3)
//     {
//       analogWrite(motor, 255 - frequency);
//     }
//     else
//     {
//       ledcWrite(motor, frequency);
//     }
//   }
// }

// // 　TODO: Setup関数内で初期値を設定させるので、電源投下時を0として計算させる。
// // 　      もしくは、回路を変更させずに絶対値にするのもあり。
// double errorRange = 2; // 2度の誤差を許容する

// void loop()
// {
//   stop();
//   delay(200);
//   foward();
//   delay(1000);
//   stop();
//   delay(200);
//   right();
//   delay(1000);
//   stop();
//   delay(200);
//   left();
//   delay(1000);
//   stop();
//   delay(200);
//   back();
//   delay(1000);

//   // // disconnecting
//   // if (!deviceConnected && oldDeviceConnected)
//   // {
//   //   delay(500);                  // give the bluetooth stack the chance to get things ready
//   //   pServer->startAdvertising(); // restart advertising
//   //   Serial.println("start advertising");
//   //   oldDeviceConnected = deviceConnected;
//   // }
//   // // connecting
//   // if (deviceConnected && !oldDeviceConnected)
//   // {
//   //   // do stuff here on connecting
//   //   oldDeviceConnected = deviceConnected;
//   // }

//   // // このブロックはisolateさせて、真っ直ぐにさせる精度を上げるかも
//   // jy901.GetAngle();
//   // angle = (float)jy901.stcAngle.Angle[2] / 32768 * 180;
//   // /*ここからいけるかわからん*/
//   // pCharacteristic->setValue(angle);
//   // pCharacteristic->notify();
//   // /*ここまで*/
//   // Serial.print("angle:");
//   // Serial.println(angle);

//   // if (angle > errorRange)
//   // {
//   //   while (angle > errorRange)
//   //   {
//   //     right();
//   //   }

//   //   Serial.println("right");
//   // }
//   // else if (angle < errorRange * -1)
//   // {
//   //   while (angle < errorRange * -1)
//   //   {
//   //     left();
//   //   }
//   //   Serial.println("left");
//   // }
//   // else
//   // {
//   //   forword();
//   //   Serial.println("forword");
//   // }

//   // delay(400);
//   // digitalWrite(2, HIGH);
//   // delay(400);
//   // digitalWrite(2, LOW);
// }

#include <Arduino.h>

TaskHandle_t thp[1];

// // モーター設定
// int motorPin[] = {12, 14, 26, 27};

// #define HIGH 200
// #define LOW 55

// // モーター制御権限緊急停止に使う予定 (-1はマスター)
// int privilegeCore = 1;

// // 　モーター指示
// int order = 0;

// void stop(int core)
// {
//   if (privilegeCore != core)
//     return;
//   ledcWrite(0, 128);
//   ledcWrite(1, 120);
//   ledcWrite(2, 130);
//   ledcWrite(3, 118);
// }
// void foward(int core)
// {
//   if (privilegeCore != core)
//     return;
//   ledcWrite(0, LOW);
//   ledcWrite(1, HIGH);
//   ledcWrite(2, HIGH);
//   ledcWrite(3, LOW);
// }
// void back(int core)
// {
//   if (privilegeCore != core)
//     return;
//   ledcWrite(0, HIGH);
//   ledcWrite(1, LOW);
//   ledcWrite(2, LOW);
//   ledcWrite(3, HIGH);
// }
// void left(int core)
// {
//   if (privilegeCore != core)
//     return;
//   ledcWrite(0, LOW);
//   ledcWrite(1, LOW);
//   ledcWrite(2, HIGH);
//   ledcWrite(3, HIGH);
// }
// void right(int core)
// {
//   if (privilegeCore != core)
//     return;
//   ledcWrite(0, HIGH);
//   ledcWrite(1, HIGH);
//   ledcWrite(2, LOW);
//   ledcWrite(3, LOW);
// }

// void turnl(int core, double angle)
// {
//   if (privilegeCore != core)
//     return;
//   Serial.println("turnl:" + String(abs(angle)));
//   ledcWrite(0, LOW + (abs(angle) / 20));
//   ledcWrite(1, LOW + (abs(angle) / 20));
//   ledcWrite(2, LOW + (abs(angle) / 20));
//   ledcWrite(3, LOW + (abs(angle) / 20));
// }
// void turnr(int core, double angle)
// {
//   if (privilegeCore != core)
//     return;
//   Serial.println("turnr:" + String(angle));
//   ledcWrite(0, HIGH - (angle / 20));
//   ledcWrite(1, HIGH - (angle / 20));
//   ledcWrite(2, HIGH - (angle / 20));
//   ledcWrite(3, HIGH - (angle / 20));
// }

void loop()
{
  detectOrangeColor();
  delay(10000);
  // メインループ (core 0)
  // analogRead(15)がラインセンサーの値
  // delay(50);
  // if (analogRead(15) >= 35)
  // {
  //   Serial.println(String(order));
  //   switch (order)
  //   {
  //   case 0:
  //     order = 1;
  //     break;

  //   case 1:
  //     order = 0;
  //     break;

  //   case 2:
  //     order = 3;
  //     break;

  //   case 3:
  //     order = 2;
  //     break;

  //   default:
  //     Serial.println("something wrong!");
  //     order = 1;
  //     break;
  //   }
  // }
  // else
  // {
  //   order - 1;
  // }
}

void Core0a(void *args)
{
  while (1)
  {
    // サブで実行するプログラム (core 1)
    // if (order == 0)
    // {
    //   foward(1);
    // }
    // else if (order == 1)
    // {
    //   back(1);
    // }
    // else if (order == 2)
    // {
    //   left(1);
    // }
    // else if (order == 3)
    // {
    //   right(1);
    // }
    // else
    // {
    //   // ボールに向かわせる
    //   order = 0;
    // }
  }
}

void setup()
{
  initCAM();
  // for (int i = 0; i < 4; i++)
  // {
  //   pinMode(motorPin[i], OUTPUT);
  //   ledcSetup(i, 169000, 8); // 100kHz, 8bit(256段階)
  //   ledcAttachPin(motorPin[i], i);
  // }

  // Serial.print("jyro setupping....");
  // Wire.setPins(21, 22);
  // jy901.StartIIC();
  // Serial.println("done");

  // pinMode(2, OUTPUT);

  Serial.begin(115200);
  Serial.println("start");
  for (int i = 0; i < 10; i++)
  {
    Serial.println("");
  }
  // xTaskCreatePinnedToCore(Core0a, "Core0a", 4096, NULL, 3, &thp[0], 0);
  // stop(1);
  // delay(500);
  // jy901.GetAngle();
  // double angle = (float)jy901.stcAngle.Angle[2] / 32768 * 180;
  // if (angle > 10)
  // {
  //   turnl(1, angle);
  // }
  // else if (angle < -10)
  // {
  //   turnr(1, angle);
  // }
  // else
  // {
  //   order = 0;
  // }

  // digitalWrite(2, HIGH);
  // delay(500);
  // digitalWrite(2, LOW);
}
