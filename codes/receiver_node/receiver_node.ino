
#include <HardwareSerial.h>

HardwareSerial E22Serial(1); // UART1 for LoRa E22

void setup() {
  Serial.begin(115200);
  E22Serial.begin(9600, SERIAL_8N1, 16, 17); // RX2, TX2

  Serial.println("LoRa Receiver Ready");
}

void loop() {
  if (E22Serial.available()) {
    String msg = E22Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(msg);
  }
}
