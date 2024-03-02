#include <Arduino.h>
#include <pins.h>

#include <line_sensor.h>
#include <ultrasonic_sensor.h>

HardwareSerial Serial3(UART_RX_PIN, UART_TX_PIN);

void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  delay(2000);
  Serial.print("Starting setup...");
  setupLineSensor();
  Serial.println("done");
  Serial.print("Starting UART...");
  Serial3.begin(9600);
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

void loop()
{
  int front = readUltrasonicSensor(0);
  Serial.print("Front: ");
  Serial.println(front);
  delay(200);
}
