/* This project fires six MaxBotix I2CXL-MaxSonar-EZ MB1242 sensors in sequence.
 * However, this differs from maxbotix_6test.ino in that it fires two opposing
 * sensors simultaneously (i.e. fires both sensors 1 and 4, waits, reads from
 * them, then fires 2 and 5--see crude "image" below)
 *
 *    1--2
 *  /     \
 * 6      3
 *  \    /
 *   5--4
 *
 * Adapted from original project found here: <https://www.arduino.cc/en/Tutorial/SFRRangerReader>
 *
 * @author  David Sutherin (sutherin.3@wright.edu)
 * @date    4/4/2017
 */

#include <Wire.h>

byte testAddr = 0;
int base = 112;
int i = 0;
int j;
int current, next;

void setup() {
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(57600);          // start serial communication at 9600bps
}

int reading = 0;

void loop() {
  current = base + i;
  j = (i + 3) % 6;
  next = base + j;

  // step 1: instruct sensor to read
  Wire.beginTransmission(current);
  Wire.write(byte(0x51));       // Write range command byte
  Wire.endTransmission();       // stop transmitting

  Wire.beginTransmission(next);
  Wire.write(byte(0x51));       // Write range command byte
  Wire.endTransmission();       // stop transmitting

  // step 2: wait for reading to happen
  delay(100);                   // datasheet recommends 100 ms

  // SENSOR 1 - step 3: request reading from sensor
  Wire.requestFrom(current, 2);    // request 2 bytes from slave device #112

  // step 4: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.print("Sensor "); Serial.print(current-base); Serial.print(": ");
    Serial.print(reading);   // print the reading
    Serial.print("cm\t");
  }
  else {
    Serial.println("Error reading sensor "); Serial.print(current-base);
  }

  // SENSOR 2 - step 3: request reading from sensor
  Wire.requestFrom(next, 2);    // request 2 bytes from slave device #112

  // step 4: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.print("Sensor "); Serial.print(next-base);  Serial.print(": ");
    Serial.print(reading);   // print the reading
    Serial.println("cm");
    Serial.println();
  }
  else {
    Serial.println("Error reading sensor "); Serial.print(next-base);
  }

  i = (i+1) % 6;
}
