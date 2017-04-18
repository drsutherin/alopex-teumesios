/*
 * This project tests two MaxBotix and HB100 sensor sets, fired
 * simultaneously, using an object's measured distance and speed
 * to predict its future location (i.e. /at the next reading)
 * 
 * Since the HB100 sensors aren't addressed, their power pins are 
 * connected to the Arduino's digital ports (0-3), and HIGH is
 * written to the pins when they're to be read from.  The sensors'
 * data output pins are connected via a bus into Arduino pin 8
 * 
 * @author  David Sutherin (sutherin.3@wright.edu)
 * @date    4/18/2017
 */

#include <Wire.h>
#include "FreqMeasure.h"

// General params
int i, count;
double prediction;

//Microwave/speed params
int m1_pin, m2_pin;
#define NUM_READS 5
float spd1;
float spd2;
double sum1 = 0;
double sum2 = 0;

// Ultrasonic/distance params
#define BASE 112
int u1_addr, u2_addr;
bool dir = false;
int reading1 = 0;
int reading2 = 0;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  FreqMeasure.begin();
  pinMode(m1_pin,OUTPUT);
  pinMode(m2_pin,OUTPUT);
  count = 0;
  Serial.println("Alopex Teumesios ADAM Test");
}

void loop() {
  i = 0;
  u1_addr = (BASE + count);
  u2_addr = u1_addr + ((count + 2) % 4);
  m1_pin = count;
  m2_pin = (count + 2) % 4;
  
  Serial.println("DEBUG: Start");
  // Start ultrasonic sensor reading
  Wire.beginTransmission(u1_addr);
  Wire.write(byte(0x51));
  byte error1 = Wire.endTransmission();  

  Wire.beginTransmission(u2_addr);
  Wire.write(byte(0x51));
  byte error2 = Wire.endTransmission();
  
  // Get NUM_READS readings from microwave sensors
  while (i < NUM_READS) {
    if (FreqMeasure.available()) {
      // Read from microwave sensor #1
      digitalWrite(m1_pin,HIGH);
      sum1 = sum1 + FreqMeasure.read();
      digitalWrite(m1_pin,LOW);
      
      // Read from microwave sensor #2
      digitalWrite((m2_pin),HIGH);
      sum2 = sum2 + FreqMeasure.read();
      digitalWrite(m2_pin,LOW);
      delay(50);\
      
      // Only increment if reading was successful
      // Note: could cause problems w/ prediction if readings take too long, consider 'for' loop instead
      i++;
    }
  }

  // Use average of microwave sensor readings to calculate speed
  float frequency1 = FreqMeasure.countToFrequency(sum1/NUM_READS);
  spd1 = frequency1 * 1.425;  // result in cm/s
  Serial.print("SENSOR 1\tSpeed: ");
  Serial.print(spd1);
  Serial.print("cm/s\t");
  sum1 = 0;

  float frequency2 = FreqMeasure.countToFrequency(sum2/NUM_READS);
  spd2 = frequency2 * 1.425;  // result in cm/s
  Serial.print("SENSOR 2\tSpeed: ");
  Serial.print(spd2);
  Serial.println("cm/s\t");
  sum2 = 0;
  
  // Request reading from ultrasonic sensor #1
  Wire.requestFrom(u1_addr, 2);

  // Receive reading from sensor
  if (2 <= Wire.available()) {
    reading1 = Wire.read();      // receive high byte (overwrites previous reading)
    reading1 = reading1 << 8;     // shift high byte to be high 8 bits
    reading1 |= Wire.read();     // receive low byte as lower 8 bits
    Serial.print("SENSOR 1\tRange: ");
    Serial.print(reading1);   // print the reading
    Serial.print(" cm\t");
  }

  // Request reading from ultrasonic sensor #2
  Wire.requestFrom(u2_addr, 2);

  // Receive reading from sensor
  if (2 <= Wire.available()) {
    reading2 = Wire.read();      // receive high byte (overwrites previous reading)
    reading2 = reading2 << 8;     // shift high byte to be high 8 bits
    reading2 |= Wire.read();     // receive low byte as lower 8 bits
    Serial.print("SENSOR 2\tRange: ");
    Serial.print(reading2);   // print the reading
    Serial.println(" cm\t");
  }
  
  // Calculate prediction for location @ next reading
  Serial.print("SENSOR 1\tPREDICTED LOCATION: ");
  // prediction = cm; reading = cm; .25 = sec; spd = cm/s
  prediction = reading1 - (.25 * spd1);
  Serial.print(prediction);
  Serial.println("cm");

  Serial.print("SENSOR 2\tPREDICTED LOCATION: ");
  // prediction = cm; reading = cm; .25 = sec; spd = cm/s
  prediction = reading2 - (.25 * spd2);
  Serial.print(prediction);
  Serial.println("cm");
  Serial.println();

  count = (count + 1) % 4;
}
