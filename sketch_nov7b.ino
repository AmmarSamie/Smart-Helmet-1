#include <SoftwareSerial.h>
SoftwareSerial BT(2, 3); // RX | TX

#define relay 4
#define switch1 A0
#define switch2 A1

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(relay, OUTPUT);
  pinMode(switch1, INPUT_PULLUP);
  pinMode(switch2, INPUT_PULLUP);

  digitalWrite(relay, LOW);
  Serial.println("Slave ready...");
}

void loop() {
  // ===== Receive data from Master =====
  if (BT.available()) {
    String data = BT.readStringUntil('\n');
    Serial.println("Received: " + data);

    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      int smoke = data.substring(0, commaIndex).toInt();
      int limit = data.substring(commaIndex + 1).toInt();

      if (smoke == 1 && limit == 0) {
        digitalWrite(relay, HIGH);  // Relay ON
        Serial.println("Relay ON");
      } else {
        digitalWrite(relay, LOW);   // Relay OFF
        Serial.println("Relay OFF");
      }
    }
  }

  // ===== Send Switch Data to Master =====
  int s1 = digitalRead(switch1);
  int s2 = digitalRead(switch2);

  // 1 = not pressed (because of INPUT_PULLUP)
  // 0 = pressed
  String switchData = String(s1) + "," + String(s2);
  BT.println(switchData);
  Serial.println("Sent Switch Data: " + switchData);

  delay(80); // send every second
}