#include <Arduino.h>

// ADXL335 analog pins connected to ESP32
const int xPin = 39;
const int yPin = 36;
const int zPin = 35;

// ADXL335 setup values
const float zeroG = 1.65;
const float scale = 0.330;

void setup() {
  Serial.begin(115200); // Start serial communication
  // Initialize analog pins
  analogReadResolution(12); // ESP32 uses 12 bits
}

void loop() {
  // Read raw values
  int xRaw = analogRead(xPin);
  int yRaw = analogRead(yPin);
  int zRaw = analogRead(zPin);

  // Convert raw readings to voltage
  float xVoltage = (xRaw / 4095.0) * 3.3; // Convert analog reading to voltage
  float yVoltage = (yRaw / 4095.0) * 3.3;
  float zVoltage = (zRaw / 4095.0) * 3.3;

  // Convert voltage to acceleration
  float xAccel = (xVoltage - zeroG) / scale;
  float yAccel = (yVoltage - zeroG) / scale;
  float zAccel = (zVoltage - zeroG) / scale;

  // Output the acceleration values to the serial monitor
  Serial.print("X: ");
  Serial.print(xAccel, 3); // 3 decimal places for accuracy
  Serial.print(" g, Y: ");
  Serial.print(yAccel, 3);
  Serial.print(" g, Z: ");
  Serial.print(zAccel, 3);
  Serial.println(" g");

  delay(100); // Delay for a bit to not flood the serial monitor
}
