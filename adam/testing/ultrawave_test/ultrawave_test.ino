#include <Wire.h>
#include "FreqMeasure.h"

// General params
int i = 0;
double prediction;

//Microwave/speed params
float spd;
double sum = 0;
int count = 0;

// Ultrasonic/distance params
#define SENSOR_ADDR 112
int reading = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  FreqMeasure.begin();
  Serial.println("Alopex Teumesios ADAM Test");
}

void loop() {
  if (i == 0) {
    Wire.beginTransmission(SENSOR_ADDR);  // Transmit to device #224 (default for MaxBotix I2CXL)
    Wire.write(byte(0x51));       // Write range command byte
    Wire.endTransmission();       // stop transmitting

    Serial.println("DEBUG: Ultrasonic start");
  }
  
  // put your main code here, to run repeatedly:
  uint8_t avail = FreqMeasure.available();
  if (avail) {
    Serial.println("DEBUG: Available");
    sum = sum + FreqMeasure.read();
    count = count + 1;
    //Serial.print("DEBUG: Count = "); Serial.println(count);
    delay(50);
    if (count > 6) {
      //Serial.println("DEBUG: Count > 30");
      float frequency = FreqMeasure.countToFrequency(sum/count);
      spd = frequency * 1.425;  // result in cm/s
      //Serial.print(frequency);
      //Serial.print("Hz\t");
      Serial.print("Speed: ");
      Serial.print(spd);
      Serial.print("cm/s\t");
      sum = 0;
      count = 0;
    }
  }
  else {
    Serial.println("DEBUG: FreqMeasure not Available");
  }
  if (i == 5) {
    // Request reading from ultrasonic sensor
    Wire.requestFrom(SENSOR_ADDR, 2);    // request 2 bytes from slave device #112

    // step 4: receive reading from sensor
    if (2 <= Wire.available()) { // if two bytes were received
      reading = Wire.read();  // receive high byte (overwrites previous reading)
      reading = reading << 8;    // shift high byte to be high 8 bits
      reading |= Wire.read(); // receive low byte as lower 8 bits
      //Serial.print("Time: ");
      //Serial.print(millis());
      Serial.print("Range: ");
      Serial.print(reading);   // print the reading
      Serial.print(" cm\t");
    }
    else {
      Serial.println("DEBUG: Wire not available");
    }
    // Calculate prediction for location @ next reading
    Serial.print("PREDICTED LOCATION: ");
    prediction = reading - (6 * spd);
    Serial.print(prediction);
    Serial.println("cm");
  }
  i = (i+1)%6;
}
