/*
 * This project is used to test MAVLink message transmission from one Arduino
 * to another. mavlink_pitcher.ino sends MAVLink messages from one Arduino to
 * another Arduino running mavlink_catcher.ino
 *
 * This file handles decoding and displaying ONLY MAVLink heartbeat and distance
 * sensor messages.
 *
 * Adapted from reference material found here: <http://discuss.ardupilot.org/t/mavlink-step-by-step/9629>
 *
 * @author  David Sutherin (sutherin.3@wright.edu)
 * @date    4/4/2017
 */

#include "common/mavlink.h"
#include "common/mavlink_msg_heartbeat.h"
#include "common/mavlink_msg_distance_sensor.h"

mavlink_message_t msg;
mavlink_status_t stat;
uint8_t c;

void setup() {
  Serial.begin(57600);
}

void loop() {
  if (Serial.available()) {
    c = Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &stat)) {
      Serial.print("Message ID: ");
      Serial.println(msg.msgid);

      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // 0
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg,&hb);

          Serial.println("HEARTBEAT MESSAGE");
          Serial.print("Time: ");       Serial.println(millis());
          Serial.print("Mode: ");       Serial.println(hb.custom_mode);
          Serial.print("Type: ");       Serial.println(hb.type);
          Serial.print("Autopilot: ");  Serial.println(hb.autopilot);
          Serial.print("Base Mode: ");  Serial.println(hb.base_mode);
          Serial.print("Status: ");     Serial.println(hb.system_status);
          Serial.print("MAV Version: ");Serial.println(hb.mavlink_version);\
          Serial.println();
          break;

        case MAVLINK_MSG_ID_DISTANCE_SENSOR: //132
          mavlink_distance_sensor_t dist;
          mavlink_msg_distance_sensor_decode(&msg,&dist);

          Serial.println("DISTANCE SENSOR MESSAGE");
          Serial.print("Time: ");       Serial.println(millis());
          Serial.print("Min Dist: ");   Serial.println(dist.min_distance);
          Serial.print("Max Dist: ");   Serial.println(dist.max_distance);
          Serial.print("Current: ");    Serial.println(dist.current_distance);
          Serial.print("Sensor type: ");Serial.println(dist.type);
          Serial.print("Comp ID: ");    Serial.println(dist.id);
          Serial.print("Orientation: ");Serial.println(dist.orientation);
          Serial.println();
          break;

      }
    }
  }

}
