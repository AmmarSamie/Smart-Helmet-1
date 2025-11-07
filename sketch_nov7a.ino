#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ===== Bluetooth =====
SoftwareSerial BT(3, 2); // RX | TX

// ===== GPS =====
SoftwareSerial GPS(7, 6); // RX | TX
TinyGPSPlus gps;

// ===== GSM =====
SoftwareSerial GSM(8, 9); // RX | TX (choose free pins)

// ===== MPU6050 =====
Adafruit_MPU6050 mpu;

// ===== Other sensors =====
#define mq2 A0
#define limitSwitch A1

#define trig1 5
#define echo1 4
#define trig2 11
#define echo2 10

#define led1 12
#define led2 13
#define led3 A2
#define led4 A3

long duration1, distance1;
long duration2, distance2;
String inputData = "";

unsigned long previousMillis = 0;
const unsigned long blinkInterval = 600;
bool led3State = false;
bool led4State = false;
int s1 = 0, s2 = 0;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  GPS.begin(9600);
  GSM.begin(9600);

  pinMode(mq2, INPUT);
  pinMode(limitSwitch, INPUT_PULLUP);

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

  // ===== MPU6050 init =====
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  Serial.println("Master ready...");
  delay(2000);
}

void loop() {
  // ===== MQ2 Sensor =====
  int smoke = digitalRead(mq2) > 0 ? 1 : 0;

  // ===== Limit Switch =====
  int limitState = digitalRead(limitSwitch) == HIGH ? 1 : 0;

  // ===== Ultrasonic 1 =====
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  duration1 = pulseIn(echo1, HIGH, 30000);
  distance1 = duration1 * 0.034 / 2;

  // ===== Ultrasonic 2 =====
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);
  duration2 = pulseIn(echo2, HIGH, 30000);
  distance2 = duration2 * 0.034 / 2;

  // ===== LED control =====
  digitalWrite(led1, distance1 < 20 ? HIGH : LOW);
  digitalWrite(led2, distance2 < 20 ? HIGH : LOW);

  // ===== Non-blocking LED Blinking =====
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;
    if (s1 == 0 && s2 == 1) {
      led3State = !led3State;
      digitalWrite(led3, led3State);
      digitalWrite(led4, LOW);
    } else if (s1 == 1 && s2 == 0) {
      led4State = !led4State;
      digitalWrite(led4, led4State);
      digitalWrite(led3, LOW);
    } else {
      digitalWrite(led3, LOW);
      digitalWrite(led4, LOW);
    }
  }

  // ===== Receive Bluetooth =====
  while (BT.available()) {
    char c = BT.read();
    if (c == '\n') {
      processData(inputData);
      inputData = "";
    } else {
      inputData += c;
    }
  }

  // ===== GPS Reading =====
  while (GPS.available()) {
    gps.encode(GPS.read());
  }

  // ===== MPU6050 Reading =====
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ===== Send Data via Bluetooth =====
  String data = String(smoke) + "," + String(limitState) + "," + String(distance1) + "," + String(distance2);
  if (gps.location.isValid()) {
    data += "," + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  } else {
    data += ",0,0";
  }
  data += "," + String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.z);

  BT.println(data);
  Serial.println("Sent: " + data);

  delay(100);
}

void processData(String data) {
  data.trim();
  int commaIndex = data.indexOf(',');
  if (commaIndex > 0) {
    s1 = data.substring(0, commaIndex).toInt();
    s2 = data.substring(commaIndex + 1).toInt();

    Serial.print("Received Switches -> S1: ");
    Serial.print(s1);
    Serial.print(" | S2: ");
    Serial.println(s2);
  }
}