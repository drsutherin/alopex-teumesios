/* This project fires six MaxBotix I2CXL-MaxSonar-EZ MB1242 sensors in sequence
 *
 * Adapted from original project found here: <https://www.arduino.cc/en/Tutorial/SFRRangerReader>
 *
 * @author  David Sutherin (sutherin.3@wright.edu)
 * @date    3/31/2017
 */

#include <Wire.h>

byte testAddr = 0;
int base = 112;
int i = 0;
int current;

void setup() {
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(57600);          // start serial communication at 9600bps
  Serial.println("S1\tS2\tS3\tS4\tS5\tS6");
}

int reading = 0;

void loop() {
  current = base + i;

  // step 1: instruct sensor to read
  Wire.beginTransmission(current);
  Wire.write(byte(0x51));       // Write range command byte
  Wire.endTransmission();       // stop transmitting

  // step 2: wait for reading to happen
  delay(100);                   // datasheet recommends 100 ms

  // step 3: request reading from sensor
  Wire.requestFrom(current, 2);    // request 2 bytes from slave device #112

  // step 4: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.print(reading);   // print the reading
    Serial.print("cm\t");
  }
  else {
    Serial.print("Error\t");
  }
  if (i == 5) {
    Serial.println();
  }
  i = (i+1) % 6;
}
