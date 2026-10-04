#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;


// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);
  Serial.println("SETUP...");

  Wire.begin(21, 22);

  if (!bmp.begin()) {
    Serial.println("BMP180 not found!");
    while (1) {
      delay(500);
    }
  }

  Serial.println("BMP180 found!");
}

void loop() {
  int32_t pressure = bmp.readPressure();
  Serial.printf("Pressure BMP180: %d Pa\r\n", pressure);
  delay(200);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}