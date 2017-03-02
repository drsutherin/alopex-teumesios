#include <FreqPeriod.h>

// General params
int i = 0;                        // Counter to determine which sensor set to fire
#define MIN_INTERVAL 100          // Minimum interval for firing an individual sensor (ms)
unsigned long last;               // Used to determine whether last firing was long enough ago
unsigned long lastFire[6];        // Used to store last firing time
int result;                       // Value to send to ArduPilot

// Ultrasonic/distance params
int uSensors[] = {0,1,2,3,4,5};   // Values correspond to ultrasonic sensor pin numbers
int dist;                         // Detected distance in cm

// Microwave/speed params
double lfrq;                      // Used for Doppler shift frequency
long int pp;                      // Not entirely sure, I think period
int spd;                          // Detected speed in cm/s

/*
 * Initial setup
 */
void setup() {
  Serial.begin(9600);
  FreqPeriod::begin();
  Serial.println("Alopex Teumesios ADAM Test");
}

/*
 * Main loop fires ultrasonic/microwave sensor pairs in sequence, predicts future location of
 * detected objects based on distance and speed, generates MAVLink messages, and transmits 
 * messages to ArduPilot
 */
void loop() {
  // Check whether enough time has passed
  if (millis() - lastFire[i] > MIN_INTERVAL)  {
    lastFire[i] = millis();
    
    // GET DISTANCE READING FROM ULTRASONIC SENSOR
    dist = analogRead(uSensors[i]); // assuming this will need expanded on  

    // GET SPEED READING FROM MICROWAVE SENSOR
    // Note: Microwave reading code blatantly stolen from https://www.gitbook.com/book/kd8bxp/arduino-project-doppler-radar-speed-detection-usi/details
    pp = FreqPeriod::getPeriod();   // TODO: figure out how getPeriod knows which sensor to check given multiple and/or adjust to do so
    if (pp) {
      Serial.print ("Period: ");
      Serial.print(pp);
      Serial.print("\t");
      // Calculate frequency of Doppler shift
      Serial.print("1/16us/frequency: ");
      lfrq = 16000400.0/pp;
      Serial.print(lfrq);
      Serial.print(" Hz \t");
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
  }
}
