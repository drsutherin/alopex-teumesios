/* MaxBotix I2CXL-MaxSonar-EZ MB1242 test code
 * David Sutherin (sutherin.3@wright.edu)
 * Adapted from original project found here: <https://www.arduino.cc/en/Tutorial/SFRRangerReader>
 * Adds MAVLink DISTANCE_SENSOR messages 3/24/17
 *    Reference: <http://discuss.ardupilot.org/t/mavlink-step-by-step/9629>
 *
 */

#include <Wire.h>
#include "common/mavlink.h"
#include "common/mavlink_msg_heartbeat.h"

#define SENSOR_ADDR 112
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
  // Send heartbeat msg
  /*
  c = millis();
  if (c - t > 2000) {
    mavlink_msg_heartbeat_pack(2,1,&msg,13,0,16,0,4);
    len = mavlink_msg_to_send_buffer(buf,&msg);
    Serial.write(buf,len);
    Serial.println();
    t = c;
  }
  if (c - t > 10000) {
    orient = (orient + 1) % 8;
  }
  */
  // These three lines used if unsure what sensor's address is, change beginTransmission & requestFrom to request from testAddr
  Serial.print("Test Address: ");
  Serial.print(testAddr);
  Serial.print(" ");
  
  // step 1: instruct sensor to read
  Wire.beginTransmission(testAddr);  // Transmit to device #224 (default for MaxBotix I2CXL)
  // the address specified in the datasheet is 224 (0xE0)
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x51));       // Write range command byte
  Wire.endTransmission();       // stop transmitting

  // step 2: wait for reading to happen
  delay(100);                   // datasheet recommends 100 ms

  // step 3: request reading from sensor
  Wire.requestFrom(testAddr, 2);    // request 2 bytes from slave device #112

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
    // Increment testAddr until correct address is found
    testAddr++;
  }
}
