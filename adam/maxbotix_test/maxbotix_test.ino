/* MaxBotix I2CXL-MaxSonar-EZ MB1242 test code
 * David Sutherin (sutherin.3@wright.edu)
 * Adapted from original project found here: <https://www.arduino.cc/en/Tutorial/SFRRangerReader>
 *
 */

#include <Wire.h>

void setup() {
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);          // start serial communication at 9600bps
}

int reading = 0;

void loop() {
  // step 1: instruct sensor to read
  Wire.beginTransmission(112);  // Transmit to device #224 (default for MaxBotix I2CXL)
  Wire.write(byte(0x51));       // Write range command byte
  Wire.endTransmission();       // stop transmitting

  // step 2: wait for reading to happen
  delay(100);                   // datasheet recommends 100 ms

  // step 3: request reading from sensor
  Wire.requestFrom(112, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print("\tRange: ");
    Serial.print(reading);   // print the reading
    Serial.println(" cm");
  }
}