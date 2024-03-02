#include <Arduino.h>
#include <pins.h>

#include <line_sensor.h>
#include <ultrasonic_sensor.h>

HardwareSerial Serial3(UART_RX_PIN, UART_TX_PIN);

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  delay(2000);
  Serial.print("Starting setup...");
  setupLineSensor();
  Serial.println("done");
  Serial.print("Starting UART...");
  Serial3.begin(115200);
  Serial.println("done");
  Serial.print("Setup ultrasonic sensor...");
  setupUltrasonicSensor();
  Serial.println("done");
  int i = 0;
  while (i != 20)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
    i++;
  }
}

double front_distance = 0;
double left_distance = 0;
double right_distance = 0;
double back_distance = 0;

int front_line_sensor = 0;
int left_line_sensor = 0;
int right_line_sensor = 0;
int back_line_sensor = 0;

void loop()
{
  // front_distance = readUltrasonicSensor(0);
  // left_distance = readUltrasonicSensor(1);
  // right_distance = readUltrasonicSensor(2);
  // back_distance = readUltrasonicSensor(3);

  front_line_sensor = readLineSensor(0);
  left_line_sensor = readLineSensor(1);
  right_line_sensor = readLineSensor(2);
  back_line_sensor = readLineSensor(3);

  // String i = Serial3.readString();
  // Serial.println(i);
  if (Serial3.available())
  {
    String i = Serial3.readString();
    Serial.println(i);
    Serial3.println(String(front_line_sensor) + " " + String(left_line_sensor) + " " + String(right_line_sensor) + " " + String(back_line_sensor));
  }
}
