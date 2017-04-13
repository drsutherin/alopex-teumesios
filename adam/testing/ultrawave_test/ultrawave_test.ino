/*
 * This project is used to test a single MaxBotix and HB100 sensor set,
 * using an object's measured distance and speed to predict its
 * location at the next reading
 * 
 * @author  David Sutherin (sutherin.3@wright.edu)
 * @date    4/12/2017
 */

#include <Wire.h>
#include "FreqMeasure.h"

// General params
int i;
double prediction;

//Microwave/speed params
#define NUM_READS 10
float spd;
double sum = 0;

// Ultrasonic/distance params
#define SENSOR_ADDR 112
int reading = 0;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  FreqMeasure.begin();
  Serial.println("Alopex Teumesios ADAM Test");
}

void loop() {
  i = 0;
  Serial.println("DEBUG: Start");
  // Start ultrasonic sensor reading
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(byte(0x51));
  byte error = Wire.endTransmission();  
  Serial.println("DEBUG: Ultrasonic start");

  // Get NUM_READS readings from microwave sensors
  while (i < NUM_READS) {
    //Serial.print("DEBUG: i = "); Serial.println(i);
    if (FreqMeasure.available()) {
      //Serial.println("DEBUG: Available");
      sum = sum + FreqMeasure.read();
      delay(50);
      // Only increment if reading was successful
      // Note: could cause problems w/ prediction if readings take too long, consider 'for' loop instead
      i++;
    }
  }

  // Use average of microwave sensor readings to calculate speed
  float frequency = FreqMeasure.countToFrequency(sum/NUM_READS);
  spd = frequency * 1.425;  // result in cm/s
  Serial.print("Speed: ");
  Serial.print(spd);
  Serial.print("cm/s\t");
  sum = 0;
  
  // Request reading from ultrasonic sensor
  Wire.requestFrom(SENSOR_ADDR, 2);

  // Receive reading from sensor
  if (2 <= Wire.available()) {
    reading = Wire.read();      // receive high byte (overwrites previous reading)
    reading = reading << 8;     // shift high byte to be high 8 bits
    reading |= Wire.read();     // receive low byte as lower 8 bits
    Serial.print("Range: ");
    Serial.print(reading);   // print the reading
    Serial.print(" cm\t");
  }
  else {
    Serial.println("DEBUG: Wire not available");
  }
  
  // Calculate prediction for location @ next reading
  Serial.print("PREDICTED LOCATION: ");
  // prediction = cm; reading = cm; .25 = sec; spd = cm/s
  prediction = reading - (.25 * spd);
  Serial.print(prediction);
  Serial.println("cm");
}
