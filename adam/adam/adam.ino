#include <Wire.h>

#include <FreqPeriod.h>

// General params
int sensors[] = {1,2,3,5,6,7};                  // Values correspond to MAVLink regions that ArduPilot expects
int i = 0, j = 2;                               // Counters to determine which sensor set to fire
#define MIN_INTERVAL 100                        // Minimum interval for firing an individual sensor (ms)
int result;                                     // Value to send to ArduPilot

// Ultrasonic/distance params
int uSensors[] = {112,113,114,115,116,117};     // Values correspond to ultrasonic sensor I2C numbers
int dist, dist2;                                // Detected distances in cm

// Microwave/speed params
double lfrq;                                    // Used for Doppler shift frequency
long int pp;                                    // Not entirely sure, I think period
int spd;                                        // Detected speed in cm/s

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
  // Instruct sensor to read
  Wire.beginTransmission(uSensors[i]); // Transmit to sensor
  Wire.write(byte(0x51));              // Write range command byte
  Wire.endTransmission();              // Stop transmitting

  // Wait for reading
  delay(100);                          // Datasheet recommends 100 ms

  // Request reading
  Wire.requestFrom(uSensors[i], 2);    // Request 2 bytes from sensor

  // Receive reading
  if (2 <= Wire.available()) {         // If two bytes were received
    dist = Wire.read();               // Receive high byte
    dist = dist << 8;                // Shift high byte to be high 8 bits
    dist |= Wire.read();              // Receive low byte as lower 8 bits
    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print("\tRange: ");
    Serial.print(dist);           // Print the reading
    Serial.println(" cm");
  }



  // GET SPEED READING FROM MICROWAVE SENSOR
  // Note: Microwave reading code shamelessly stolen from https://www.gitbook.com/book/kd8bxp/arduino-project-doppler-radar-speed-detection-usi/details
  pp = FreqPeriod::getPeriod();   // TODO: figure out how getPeriod knows which sensor to check given multiple and/or adjust to do so
  if (pp) {
    Serial.print ("Period: ");
    Serial.print(pp);
    Serial.print("\t");
    // Calculate frequency of Doppler shift
    Serial.print("1/16us/frequency: ");
    lfrq = 16000400.0/pp;
    Serial.print(lfrq);
    Serial.print(" Hz\t");
    // Calculate speed
    spd = lfrq * 1.425;  // result in cm/s; spd is an int so decimal is truncated
    Serial.print(spd);
    Serial.println( " cm/s ");
  }

  // Make sure readings are valid
  if (dist > 0 && spd > 0)  {
    // PREDICT OBJECT LOCATION AT NEXT FIRING
    result = dist - (spd * MIN_INTERVAL);
    if (result <= 0)  { result = 50;  } // If predicted location is <0, tell ArduPilot it's 50cm away
    // GENERATE MAVLINK MESSAGE
    // TODO: that ^
    
    // TRANSMIT MAVLINK MESSAGE
    // TODO: that ^
  }
  // Increment sensor counter
  i = (i + 1) % 6;
  j = (j + 1) % 6;
}
