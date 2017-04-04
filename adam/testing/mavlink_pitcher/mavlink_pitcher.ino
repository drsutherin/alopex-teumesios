/*
 * This project is used to test MAVLink message transmission from one Arduino
 * to another. mavlink_pitcher.ino sends MAVLink messages from one Arduino to
 * another Arduino running mavlink_catcher.ino
 *
 * This file continually sends DISTANCE_SENSOR messages indicating an object is
 * a given distance away, changing orientation every 10 seconds. Also sends a
 * heartbeat message every 30 seconds
 *
 * Adapted from reference material found here: <http://discuss.ardupilot.org/t/mavlink-step-by-step/9629>
 *
 * @author  David Sutherin (sutherin.3@wright.edu)
 * @date    4/4/2017
 */

#include "common/mavlink.h"
#include "common/mavlink_msg_heartbeat.h"

#define HARD_DISTANCE

// MAVLink DISTANCE_SENSOR
uint16_t min_dist = 20;                            // Minimum sensor distance in cm
uint16_t max_dist = 700;                           // Maximum sensor distance in cm
uint16_t current_dist = 0;                         // Current distance reading
uint8_t orient = 0;                                // Orientation of the sensor (0-7 represent 45 degree increments w/ 0 directly in front)

// MAVLink
unsigned long c,hbt,o,ds;
mavlink_message_t hb, dist;
uint8_t len;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];


void setup() {
  Serial.begin(57600);
  mavlink_msg_heartbeat_pack(1,158,&hb,13,0,16,0,4);
  c = 0; hbt = 0; o = 0; ds = 0;
}

void loop() {
    // Send heartbeat msg
  c = millis();
  if (c - hbt > 30000) {
//    mavlink_msg_heartbeat_pack(1,158,&hb,13,0,16,0,4);
    /**
   * @param system_id ID of this system
   * @param component_id ID of this component (e.g. 200 for IMU)
   * @param msg The MAVLink message to compress the data into
   *
   * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
   * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
   * @param base_mode System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
   * @param custom_mode A bitfield for use for autopilot-specific flags.
   * @param system_status System status flag, see MAV_STATE ENUM
   * @return length of the message in bytes (excluding serial stream start sign)

      uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
*/
//len = mavlink_msg_to_send_buffer(buf,&hb);
    //Serial.print("Heartbeat: ");
  //  Serial.write(buf,len);
    //Serial.println();
    hbt = c;
  }
  if (c - o > 10000) {
    orient = (orient + 1) % 8;
    o = c;
    //Serial.print("Orientation: ");
    //Serial.println(orient);
  }
  if (c - ds > 40) {
  /**
  * @brief Pack a distance_sensor message
  * @param system_id ID of this system
  * @param component_id ID of this component (e.g. 200 for IMU)
  * @param msg The MAVLink message to compress the data into
  *
  * @param time_boot_ms Time since system boot
  * @param min_distance Minimum distance the sensor can measure in centimeters
  * @param max_distance Maximum distance the sensor can measure in centimeters
  * @param current_distance Current distance reading
  * @param type Type from MAV_DISTANCE_SENSOR enum.
  * @param id Onboard ID of the sensor
  * @param orientation Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
  * @param covariance Measurement covariance in centimeters, 0 for unknown / invalid readings
  * @return length of the message in bytes (excluding serial stream start sign)

  static inline uint16_t mavlink_msg_distance_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                                 uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)
  */
    mavlink_msg_distance_sensor_pack(1,158,&dist,0,20,700,30,0,0,orient,0);
    len = mavlink_msg_to_send_buffer(buf,&dist);
    //Serial.print("Distance: ");
    Serial.write(buf,len);
    //Serial.println();
    ds = c;
  }

}
