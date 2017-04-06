
#include "FreqMeasure.h"

//Microwave/speed params
double lfrq1, lfrq2;                               // Used for Doppler shift frequency
long int pp1, pp2;                                 // Not entirely sure, I think period
int spd1, spd2;     // Detected speed in cm/s
double sum = 0;
int count = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  FreqMeasure.begin();
  Serial.println("Alopex Teumesios ADAM Test");
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t avail = FreqMeasure.available();
  //Serial.print("DEBUG: avail = "); Serial.println(avail);
  if (avail) {
    //Serial.println("DEBUG: Available");
    // average 30 readings together
    sum = sum + FreqMeasure.read();
    count = count + 1;
    //Serial.print("DEBUG: Count = "); Serial.println(count);
    delay(50);
    if (count > 5) {
      //Serial.println("DEBUG: Count > 30");
      float frequency = FreqMeasure.countToFrequency(sum/count);
      float spd = frequency * 1.425;  // result in cm/s
      Serial.print(frequency);
      Serial.print("Hz\t");
      Serial.print(spd);
      Serial.println("cm/s");
      sum = 0;
      count = 0;
    }
  }
  else {
    Serial.println("DEBUG: Not Available");
  }
}
