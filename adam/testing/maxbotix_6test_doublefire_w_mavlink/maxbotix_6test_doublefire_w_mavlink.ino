/* This project fires six MaxBotix I2CXL-MaxSonar-EZ MB1242 sensors in sequence.
 * However, this differs from maxbotix_6test_doublefire.ino in that it adds
 * MAVLink message transmission (sending DISTANCE_SENSOR messages via UART)
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
#include "common/mavlink.h"
#include "common/mavlink_msg_heartbeat.h"

int base = 112;
int i = 0;
int j;
int current, next;

// MAVLink DISTANCE_SENSOR
uint16_t min_dist = 20;                            // Minimum sensor distance in cm
uint16_t max_dist = 700;                           // Maximum sensor distance in cm
uint16_t current_dist = 0;                         // Current distance reading
uint8_t orient = 0;                                // Orientation of the sensor (0-7 represent 45 degree increments w/ 0 directly in front)

unsigned long c,ds;
mavlink_message_t hb, dist;
uint8_t len;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

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
    mavlink_msg_distance_sensor_pack(1,158,&dist,0,20,700,reading,0,0,i,0);
    len = mavlink_msg_to_send_buffer(buf,&dist);
    Serial.write(buf,len);
  }
  else {
    Serial.println("Error reading sensor "); Serial.print(current);
  }

  // SENSOR 2 - step 3: request reading from sensor
  Wire.requestFrom(next, 2);

  // step 4: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits

     mavlink_msg_distance_sensor_pack(1,158,&dist,0,20,700,reading,0,0,j,0);
     len = mavlink_msg_to_send_buffer(buf,&dist);
     Serial.write(buf,len);
  }
  else {
    Serial.println("Error reading sensor "); Serial.print(next);
  }
  
  i = (i+1) % 6;
}
