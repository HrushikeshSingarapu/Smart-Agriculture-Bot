
#include <HardwareSerial.h>

HardwareSerial E22Serial(1); // UART1 for LoRa E22

// Motor Pins (L298N)
#define IN1 4
#define IN2 5
#define IN3 18
#define IN4 19

#define SOIL_PIN 34              // Soil moisture analog pin
#define YELLOW_MOTOR_PIN 23      // Motor to push sensor into soil

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void moveForward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void stopBot() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(115200);
  E22Serial.begin(9600, SERIAL_8N1, 16, 17); // RX2, TX2 for LoRa

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(SOIL_PIN, INPUT);
  pinMode(YELLOW_MOTOR_PIN, OUTPUT);

  delay(1000);
  Serial.println("Bot + Sensor + LoRa Sender Ready");
}

void loop() {
  for (int row = 0; row < 4; row++) {           // 4 rows
    for (int step = 0; step < 4; step++) {      // 4 points per row
      moveForward();
      delay(2000);                               // Move forward
      stopBot();
      delay(5000);                               // Wait before sensing

      // Push sensor into soil
      digitalWrite(YELLOW_MOTOR_PIN, HIGH);
      delay(2000);

      // Read soil moisture
      int soilValue = analogRead(SOIL_PIN);
      String msg = "Soil Moisture: " + String(soilValue);

      // Send via LoRa
      E22Serial.println(msg);
      Serial.println("Sent via LoRa: " + msg);

      // Retract sensor
      digitalWrite(YELLOW_MOTOR_PIN, LOW);
      delay(1000);
    }

    // Turn at end of row
    turnRight();
    delay(1500);
    stopBot();
    delay(1000);
  }

  // Stop permanently after grid scan
  while (true) {
    stopBot();
    delay(10000);
  }
}
