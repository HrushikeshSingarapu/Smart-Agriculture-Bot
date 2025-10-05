# Smart-Agriculture-Bot

Data Transmitting Code
#include <HardwareSerial.h>

HardwareSerial E22Serial(1);  // UART1

// Motor Pins
#define IN1 4
#define IN2 5
#define IN3 18
#define IN4 19

// Second Motor Driver Pins
#define IN5 12
#define IN6 13

#define SOIL_PIN 34  // Analog input pin for sensor

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

// Second motor control
void secondMotorForward() {
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
}

void secondMotorBackward() {
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
}

void secondMotorStop() {
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
}

void setup() {
  Serial.begin(115200);
  E22Serial.begin(9600, SERIAL_8N1, 16, 17);  // RX2, TX2

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
  pinMode(SOIL_PIN, INPUT);

  delay(1000);
  Serial.println("Bot + Sensor + LoRa Sender Ready");
}

void loop() {
  // 1. Move Forward
  moveForward();
  delay(2000);

  // 2. Move Backward
  moveBackward();
  delay(2000);

  // 3. Turn Lefttt
  turnLeft();
  delay(1500);

  // 4. Turn Right
  turnRight();
  delay(1500);

  // 5. Stop
  stopBot();
  delay(1000);

  // 6. Read Soil Sensor
  int soilValue = analogRead(SOIL_PIN);

  // 7. Send over LoRa
  String msg = "Soil Moisture: " + String(soilValue);
  E22Serial.println(msg);
  Serial.println("Sent via LoRa: " + msg);

  delay(3000);  // Short wait before repeating
}



Data Receiving Code
#include <HardwareSerial.h>

HardwareSerial E22Serial(1);  // UART1

void setup() {
  Serial.begin(115200);
  E22Serial.begin(9600, SERIAL_8N1, 16, 17);  // RX2, TX2
  Serial.println("LoRa Receiver Ready");
}

void loop() {
  if (E22Serial.available()) {
    String msg = E22Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(msg);
  }
}


  
