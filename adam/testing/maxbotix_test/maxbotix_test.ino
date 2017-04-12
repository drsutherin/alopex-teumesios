/* MaxBotix I2CXL-MaxSonar-EZ MB1242 test code
 * David Sutherin (sutherin.3@wright.edu)
 * Adapted from original project found here: <https://www.arduino.cc/en/Tutorial/SFRRangerReader>
 */

#include <Wire.h>

#define SENSOR_ADDR 112
int reading = 0;

void setup() {
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(57600);          // start serial communication at 9600bps
}

void loop() {
  
  // step 1: instruct sensor to read
  Wire.beginTransmission(SENSOR_ADDR);  // Transmit to device #224 (default for MaxBotix I2CXL)
  // the address specified in the datasheet is 224 (0xE0)
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x51));       // Write range command byte
  Wire.endTransmission();       // stop transmitting

  // step 2: wait for reading to happen
  delay(100);                   // datasheet recommends 100 ms

  // step 3: request reading from sensor
  Wire.requestFrom(SENSOR_ADDR, 2);    // request 2 bytes from slave device #112

  // step 4: receive reading from sensor
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
  else {
    Serial.println("Sensor read error");
  }
}
