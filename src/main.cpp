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
  double target_angle = angle + 90;
  rotateLeft(200);
  while (angle < target_angle)
  {
    power_dribble();
    angle = get_angle();
  }
  kick();
  delay(30);
  stop();
  dribble();
}

/*
0: forward
1: backward
*/
int stats = 0;
int times = 0;

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
    delay(10);
  }
  Serial.println("Start signal received!");
}

void loop()
{
  face_forward(0);
  makao();
  delay(5000);
  moveBackward();
  delay(300);
  stop();
  delay(1000);
  stop();

  angle = get_angle() * -1;
  front_line_sensor = readLineSensor(0);
  left_line_sensor = readLineSensor(1);
  right_line_sensor = readLineSensor(2);
  back_line_sensor = readLineSensor(3);

  Serial.println(String(angle) + "," + String(motor_power[0]) + "," + String(motor_power[1]) + "," + String(motor_power[2]) + "," + String(motor_power[3]));

  // front_distance = readUltrasonicSensor(0);
  // left_distance = readUltrasonicSensor(1);
  // right_distance = readUltrasonicSensor(2);
  // back_distance = readUltrasonicSensor(3);

  // uart
  if (Serial3.available())
  {
    // String i = Serial3.readString();
    // Serial.println(i);
    Serial3.println(String(angle) + "," + String(motor_power[0]) + "," + String(motor_power[1]) + "," + String(motor_power[2]) + "," + String(motor_power[3]));
  }
  // moveWith_angleCorrection(angle);
  delay(100);

  // Serial.println(String(front_distance) + "cm\t" + String(left_distance) + "cm\t" + String(right_distance) + "cm\t" + String(back_distance) + "cm\t");
  // Serial.println(String(front_line_sensor) + " " + String(left_line_sensor) + " " + String(right_line_sensor) + " " + String(back_line_sensor));

  // if (stats == 0)
  // {
  //   moveWith_angleCorrection(angle);
  //   if (front_line_sensor > 1000)
  //   {
  //     stats = 1;
  //   }
  // }
  // else
  // {
  //   moveBackward();
  //   if (back_line_sensor > 1000)
  //   {
  //     stats = 0;
  //   }
  // }
}