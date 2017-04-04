#include <Wire.h>

#include <FreqPeriod.h>

// General params
// TODO: Get rid of the 6-sensor/8-region weirdness; will change ArduPilot sensor regions to be 60deg instead of 45
int sensors[] = {1,2,3,5,6,7};                     // Values correspond to MAVLink regions that ArduPilot expects
int i = 0, j = 2;                                  // Counters to determine which sensor set to fire
#define MIN_INTERVAL 100                           // Minimum interval for firing an individual sensor (ms)
uint16_t result;                                   // Value to send to ArduPilot
unsigned long t_microStart, t_microTotal;          // Time microwave sensors are fired & duration of firing

// Ultrasonic/distance params
int uSensors[] = {112,113,114,115,116,117};        // Values correspond to ultrasonic sensor I2C numbers
int dist1, dist2;                                  // Detected distances in cm

// Microwave/speed params
double lfrq1, lfrq2;                               // Used for Doppler shift frequency
long int pp1, pp2;                                 // Not entirely sure, I think period
int spd1, spd2;                                    // Detected speed in cm/s

// MAVLink params
// Header/footer
uint8_t p_start = 0xfe;                            // Start of new packet
uint8_t p_size = 14;                               // Size of payload
uint8_t p_seq = 0;                                 // Sequence counter (increment @ each message)
uint8_t p_sysID = 255;                             // ID of sending system (manually assigned 255, not sure if that's correct)
uint8_t p_compID = 255;                            // ID of the sending component (manually assigned 255, not sure if that's correct either)
uint8_t p_msgID = 132;                             // Message type identifier.  132 is DISTANCE_SENSOR
uint16_t p_check = 0;                              // Checksum calculated for each message (ITU X.25/SAE AS-4 hash, excluding p_start)

// DISTANCE_SENSOR
uint16_t min_dist = 20;                            // Minimum sensor distance in cm
uint16_t max_dist = 700;                           // Maximum sensor distance in cm
uint16_t current_dist = 0;                         // Current distance reading
uint8_t orient = 0;                                // Orientation of the sensor (0-7 represent 45 degree increments w/ 0 directly in front)

// Parameters ignored by ArduPilot, don't waste memory declaring them all. Just use hard-coded 0
//uint32_t time_boot_ms = 0;                         // Milliseconds since boot (ignored by ArduPilot)
//uint8_t type = 0;                                  // "Type from MAV_DISTANCE_SENSOR enum" (ignored by ArduPilot)
//uint8_t id = 0;                                    // "Onboard ID of the sensor" (ignored by ArduPilot)
//uint8_t covar = 0;                                 // "Measurement covariance in centimeters, 0 for unknown / invalid readings" (ignored by ArduPilot)

/*
 * Initial setup
 */
void setup() {
  Serial.begin(9600);
  FreqPeriod::begin();
  Wire.begin();
  Serial.println("Alopex Teumesios ADAM Test");
}

/*
 * Main loop fires ultrasonic/microwave sensor pairs in sequence, predicts future location of
 * detected objects based on distance and speed, generates MAVLink messages, and transmits 
 * messages to ArduPilot
 */
void loop() {
  
  // GET DISTANCE READING FROM ULTRASONIC SENSORS
  // Instruct sensors to read
  Wire.beginTransmission(uSensors[i]); // Transmit to sensor I
  Wire.write(byte(0x51));              // Write range command byte
  Wire.endTransmission();              // Stop transmitting
  Wire.beginTransmission(uSensors[j]); // Transmit to sensor J
  Wire.write(byte(0x51));              // Write range command byte
  Wire.endTransmission();              // Stop transmitting
  
  // Wait for reading
  t_microStart = millis();
  
  
  // GET SPEED READING FROM MICROWAVE SENSORS
  // Note: Microwave reading code shamelessly stolen from https://www.gitbook.com/book/kd8bxp/arduino-project-doppler-radar-speed-detection-usi/details
  
  // TODO: figure out how getPeriod knows which sensor to check given multiple and/or adjust to do so
  pp1 = FreqPeriod::getPeriod();
  if (pp1) {
    Serial.print ("Period: ");
    Serial.print(pp1);
    Serial.print("\t");
    // Calculate frequency of Doppler shift
    Serial.print("1/16us/frequency: ");
    lfrq = 16000400.0/pp1;
    Serial.print(lfrq1);
    Serial.print(" Hz\t");
    // Calculate speed
    spd1 = lfrq1 * 1.425;  // result in cm/s; spd is an int so decimal is truncated
    Serial.print(spd1);
    Serial.println( " cm/s ");
  }

  pp2 = FreqPeriod::getPeriod();
  if (pp2) {
    Serial.print ("Period: ");
    Serial.print(pp2);
    Serial.print("\t");
    // Calculate frequency of Doppler shift
    Serial.print("1/16us/frequency: ");
    lfrq = 16000400.0/pp2;
    Serial.print(lfrq2);
    Serial.print(" Hz\t");
    // Calculate speed
    spd2 = lfrq2 * 1.425;  // result in cm/s; spd is an int so decimal is truncated
    Serial.print(spd2);
    Serial.println( " cm/s ");
  }

  // Make sure at least 100ms have passed before requesting ultrasonic sensor readings
  t_microTotal = millis() - t_microStart;
  if (t_microTotal < 100) {
    delay(100 - t_microTotal);
  }

  // Request reading from sensor I
  Wire.requestFrom(uSensors[i], 2);    // Request 2 bytes from sensor

  // Receive reading from sensor I
  if (2 <= Wire.available()) {         // If two bytes were received
    dist1 = Wire.read();               // Receive high byte
    dist1 = dist1 << 8;                // Shift high byte to be high 8 bits
    dist1 |= Wire.read();              // Receive low byte as lower 8 bits
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print("\tTime: ");
    Serial.print(millis());
    Serial.print("\tRange: ");
    Serial.print(dist1);           // Print the reading
    Serial.println(" cm");
  }

  // Request reading from sensor J
  Wire.requestFrom(uSensors[j], 2);    // Request 2 bytes from sensor A

  // Receive reading from sensor J
  if (2 <= Wire.available()) {         // If two bytes were received
    dist2 = Wire.read();               // Receive high byte
    dist2 = dist2 << 8;                // Shift high byte to be high 8 bits
    dist2 |= Wire.read();              // Receive low byte as lower 8 bits
    Serial.print("Sensor ");
    Serial.print(j);
    Serial.print("\tTime: ");
    Serial.print(millis());
    Serial.print("\tRange: ");
    Serial.print(dist2);           // Print the reading
    Serial.println(" cm");
  }


  // SEND SENSOR I READING TO ARDUPILOT
  if (dist1 > 0 && spd1 > 0)  {
    // PREDICT OBJECT LOCATION AT NEXT FIRING
    result = dist1 - (spd1 * MIN_INTERVAL);
    if (result <= 0)  { result = 20;  } // If predicted location is <0, tell ArduPilot it's 20cm away
    // GENERATE MAVLINK MESSAGE
    // TODO: that ^
    
    // TRANSMIT MAVLINK MESSAGE
    // TODO: that ^
  }
  
  // SEND SENSOR J READING TO ARDUPILOT
  if (dist2 > 0 && spd2 > 0)  {
    // PREDICT OBJECT LOCATION AT NEXT FIRING
    result = dist1 - (spd * MIN_INTERVAL);
    if (result <= 0)  { result = 20;  } // If predicted location is <0, tell ArduPilot it's 20cm away
    // GENERATE MAVLINK MESSAGE
    // TODO: that ^
    
    // TRANSMIT MAVLINK MESSAGE
    // TODO: that ^
  }
  // Increment sensor counter
  i = (i + 1) % 6;
  j = (j + 1) % 6;
}
