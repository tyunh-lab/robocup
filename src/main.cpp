#include <Arduino.h>
#include <pins.h>

// 駆動系
#include <kicker.h>
#include <dribble.h>
#include <motor.h>

// センサー系
#include <line_sensor.h>
#include <ultrasonic_sensor.h>
// #include <JY901.h>
// #include <JY901_dfs.h>
#include <jyro.h>

// 定形動作
// #include <action.h>

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
void face_forward(void)
{
  angle = get_angle();
  while (!(angle <= 12.5 && angle >= -12.5))
  {
    angle = get_angle();
    if (angle >= 12.5)
    {
      rotateRight(angle / 2);
    }
    else if (angle <= -12.5)
    {
      rotateLeft(abs(angle) / 2);
    }
  }
  stop();
}

int melody[] = {
    165,
    220,
    330,
    494,
    659,
    880,
    1661,
};

int noteDurations[] = {
    6, 6, 6, 6, 6, 6, 2};

void playMelody(int buzzerPin)
{
  for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++)
  {
    int duration = 1000 / noteDurations[i];
    tone(buzzerPin, melody[i], duration);

    // 周波数と長さの後に一定の遅延
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);

    // ブザーを停止
    noTone(buzzerPin);
  }
}

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

void setup()
{
  // Serial.begin(9600);
  Serial.begin(115200);
  // Serial.begin(921600);
  pinMode(LED_PIN1, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  pinMode(LED_PIN2, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TOGGLE_SWITCH_PIN, INPUT);
  pinMode(TACT_SWITCH_L_PIN, INPUT);
  pinMode(TACT_SWITCH_M_PIN, INPUT);
  pinMode(TACT_SWITCH_R_PIN, INPUT);
  pinMode(TINY_SWITCH_PIN, INPUT);
  Serial.print("Setup line sensor...");
  setupLineSensor();
  Serial.println("done");
  Serial.print("Setup motor...");
  initMotor();
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
  stop();
  delay(100);
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
  attachInterrupt(digitalPinToInterrupt(TACT_SWITCH_R_PIN), on_led, RISING);
  attachInterrupt(digitalPinToInterrupt(TINY_SWITCH_PIN), kick, RISING);

  playMelody(BUZZER_PIN);
  delay(200);
  // dribble();
}

void loop()
{
  // face_forward();
  // makao();
  // delay(5000);
  // moveBackward();
  // delay(300);
  // stop();
  // delay(1000);

  // front_distance = readUltrasonicSensor(0);
  // left_distance = readUltrasonicSensor(1);
  // right_distance = readUltrasonicSensor(2);
  // back_distance = readUltrasonicSensor(3);

  front_line_sensor = readLineSensor(0);
  left_line_sensor = readLineSensor(1);
  right_line_sensor = readLineSensor(2);
  back_line_sensor = readLineSensor(3);
  delay(300);

  if (Serial3.available())
  {
    String i = Serial3.readString();
    Serial.println(i);
    Serial3.println(String(front_line_sensor) + " " + String(left_line_sensor) + " " + String(right_line_sensor) + " " + String(back_line_sensor));
  }

  // dribble();
  // delay(1000);
  // kick();
  // delay(300);
  // stop_dribble();
  // Serial.println(analogRead(BALL_SENSOR_PIN));
  // delay(10);
  // Serial.println(String(front_distance) + "cm\t" + String(left_distance) + "cm\t" + String(right_distance) + "cm\t" + String(back_distance) + "cm\t");
}