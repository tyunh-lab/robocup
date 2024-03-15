#include <Arduino.h>
#include <pins.h>

// 駆動系
#include <kicker.h>
#include <dribble.h>
#include <motor.h>

// センサー系
#include <line_sensor.h>
#include <ultrasonic_sensor.h>
#include <jyro.h>

// 定形動作
#include <action.h>

// その他
#include <mero.h>

HardwareSerial Serial3(UART_RX_PIN, UART_TX_PIN);

double front_distance = 0;
double left_distance = 0;
double right_distance = 0;
double back_distance = 0;

int front_line_sensor = 0;
int left_line_sensor = 0;
int right_line_sensor = 0;
int back_line_sensor = 0;

double angle = 0;

float start_angle = 0;
bool isOutOfBounds = false;

void on_led(void)
{
  digitalWrite(LED_PIN1, HIGH);
  digitalWrite(LED_PIN2, HIGH);
}

void off_led(void)
{
  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);
}

void makao()
{
  angle = get_angle();
  double target_angle = angle + 45;
  rotateLeft(20);
  power_dribble();
  delay(300);
  kick();
  delay(100);
  middle_dribble();
  face_forward(angle);
  stop();
  delay(4000);
}

void setup()
{
  // Serial.begin(9600);
  Serial.begin(115200);
  // Serial.begin(921600);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TOGGLE_SWITCH_PIN, INPUT);
  pinMode(TACT_SWITCH_L_PIN, INPUT);
  pinMode(TACT_SWITCH_M_PIN, INPUT);
  pinMode(TACT_SWITCH_R_PIN, INPUT);
  pinMode(TINY_SWITCH_PIN, INPUT);

  Serial.print("Setup motor...");
  initMotor();
  stop();
  Serial.println("done");

  Serial.print("Setup line sensor...");
  setupLineSensor();
  Serial.println("done");

  Serial.print("Setup kicker...");
  initKicker();
  Serial.println("done");

  Serial.print("Setup dribbler...");
  initDribbler();
  Serial.println("done");

  Serial.print("Setup ball_sensor...");
  pinMode(BALL_SENSOR_PIN, INPUT);
  Serial.println("done");

  Serial.print("Setup ultrasonic sensor...");
  setupUltrasonicSensor();
  Serial.println("done");

  Serial.print("Setup jyro sensor...");
  jyro_setup();
  Serial.println("done");

  Serial.print("Starting UART...");
  Serial3.begin(115200);
  Serial.println("done");

  Serial.print("Setup pid...");
  pid_setup();
  Serial.println("done");

  Serial.println("Setup complete!");
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_PIN1, HIGH);
    digitalWrite(LED_PIN2, LOW);
    delay(100);
    digitalWrite(LED_PIN1, LOW);
    digitalWrite(LED_PIN2, HIGH);
    delay(100);
  }
  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);

  attachInterrupt(digitalPinToInterrupt(TACT_SWITCH_L_PIN), on_led, RISING);
  attachInterrupt(digitalPinToInterrupt(TACT_SWITCH_M_PIN), off_led, RISING);
  // attachInterrupt(digitalPinToInterrupt(TACT_SWITCH_R_PIN), face_forward, RISING);

  attachInterrupt(digitalPinToInterrupt(TOGGLE_SWITCH_PIN), system_stop, FALLING);
  attachInterrupt(digitalPinToInterrupt(TINY_SWITCH_PIN), kick, RISING);

  playMelody(BUZZER_PIN);
  // dribble();

  Serial.println("Waiting for start signal...");

  while (digitalRead(TOGGLE_SWITCH_PIN) == 0)
  {
    stop();
    Serial3.println("0,0,0,0,0");
    if (digitalRead(TACT_SWITCH_R_PIN) == 1)
    {
      Serial.println("out of bounds!");
      isOutOfBounds = true;
      delay(20);
    }
  }
  start_angle = get_angle();
  Serial.println("Start signal received");
  if (isOutOfBounds) //<-out of boundsの時は前に向けてからloopに入る
  {
    angle = start_angle - get_angle() + 180;
    Serial.println("start_angle:" + String(start_angle) + "°");
    Serial.println("current_angle:" + String(get_angle()) + "°");
    Serial.println("calced_angle:" + String(angle) + "°");
    delay(500);
    while (!(angle <= 10 && angle >= -10))
    {
      angle = start_angle - get_angle() + 180;
      if (angle > 180)
      {
        angle -= 360;
      }
      else if (angle < -180)
      {
        angle += 360;
      }
      face_forward(angle);
      Serial.println("angle:" + String(angle) + "°");
    }
  }
  Serial.println("Set up complete");

  // test motor
  // for (int i = 0; i < 4; i++)
  // {
  //   test_motor(i);
  //   Serial.println("test" + String(i) + " done");
  //   delay(1000);
  // }
  // test_motor(0, true);
}

void loop()
{
  // test_motor(-1, true);
  // makao();

  // front_line_sensor = readLineSensor(0);
  // left_line_sensor = readLineSensor(1);
  // right_line_sensor = readLineSensor(2);
  // back_line_sensor = readLineSensor(3);

  // front_distance = readUltrasonicSensor(0);
  // left_distance = readUltrasonicSensor(1);
  // right_distance = readUltrasonicSensor(2);
  // back_distance = readUltrasonicSensor(3);
  // angleの取得
  if (isOutOfBounds)
  {
    angle = start_angle - get_angle() + 180;
  }
  else
  {
    angle = start_angle - get_angle();
  }
  if (angle > 180)
  {
    angle -= 360;
  }
  else if (angle < -180)
  {
    angle += 360;
  }

  // if (front_distance <= 30)
  // {
  //   moveBackward();
  // }
  // else if (back_distance <= 30)
  // {
  //   moveForward();
  // }
  // else if (left_distance <= 30)
  // {
  //   moveRight();
  // }
  // else if (right_distance <= 30)
  // {
  //   moveLeft();
  // }

  // Serial.println(String(angle) + "," + String(motor_power[0]) + "," + String(motor_power[1]) + "," + String(motor_power[2]) + "," + String(motor_power[3]));

  // uart
  if (Serial3.available())
  {
    String i = Serial3.readString();
    Serial.println(i);
    // Serial3.println(String(angle) + "," + String(motor_power[0]) + "," + String(motor_power[1]) + "," + String(motor_power[2]) + "," + String(motor_power[3]));
  }
}

// #include <Arduino.h>
// #include <pins.h>

// HardwareSerial Serial3(UART_RX_PIN, UART_TX_PIN);

// void setup()
//{
// Serial.begin(115200);
// Serial3.begin(115200);
// Serial.println("Hello, world!");
//}

// void loop()
//  {
//   if (Serial3.available())
//   {
//     String i = Serial3.readString();
//     Serial.println(i);
//   }
//  }
