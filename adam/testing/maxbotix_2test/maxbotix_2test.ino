/* This project is designed to alternate firing two MaxBotix I2CXL-MaxSonar-EZ
 * MB1242 sensors
 * Adapted from original project found here: <https://www.arduino.cc/en/Tutorial/SFRRangerReader>
 *
 * @author David Sutherin (sutherin.3@wright.edu)
 * @date 3/24/2017
 */

#include <Wire.h>
#include "common/mavlink.h"
#include "common/mavlink_msg_heartbeat.h"

#define SENSOR1_ADDR 112
#define SENSOR2_ADDR 113
byte testAddr = 0;

// MAVLink DISTANCE_SENS OR
uint16_t min_dist = 20;                            // Minimum sensor distance in cm
uint16_t max_dist = 700;                           // Maximum sensor distance in cm
uint16_t current_dist = 0;                         // Current distance reading
uint8_t orient = 0;                                // Orientation of the sensor (0-7 represent 45 degree increments w/ 0 directly in front)

unsigned long t,c;
mavlink_message_t msg;
uint8_t len;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

void setup() {
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(57600);          // start serial communication at 9600bps
}

int reading = 0;

void loop() {

// SENSOR 1
  // step 1: instruct sensor to read
  Wire.beginTransmission(SENSOR1_ADDR);
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x51));       // Write range command byte
  Wire.endTransmission();       // stop transmitting

  // step 2: wait for reading to happen
  delay(500);                   // datasheet recommends 100 ms

  // step 3: request reading from sensor
  Wire.requestFrom(SENSOR1_ADDR, 2);    // request 2 bytes from slave device #112

  // step 4: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.println("SENSOR 1");
    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print("\tRange: ");
    Serial.print(reading);   // print the reading
    Serial.println(" cm");
  }
  else {
    Serial.println("Sensor 1 read error");
    // Increment testAddr until correct address is found
    //testAddr++;
  }


//READ SENSOR 2
   // step 1: instruct sensor to read
  Wire.beginTransmission(SENSOR2_ADDR);
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x51));       // Write range command byte
  Wire.endTransmission();       // stop transmitting

  // step 2: wait for reading to happen
  delay(500);                   // datasheet recommends 100 ms

  // step 3: request reading from sensor
  Wire.requestFrom(SENSOR2_ADDR, 2);    // request 2 bytes from slave device #112

  // step 4: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.println("SENSOR 2");
    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print("\tRange: ");
    Serial.print(reading);   // print the reading
    Serial.println(" cm");
  }
  else {
    Serial.println("Sensor 2 read error");
  }
}
