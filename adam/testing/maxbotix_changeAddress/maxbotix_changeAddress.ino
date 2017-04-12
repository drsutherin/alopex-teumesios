/*
This project updates the address of a MaxBotix I2CXL-MaxSonar-EZ MB1242. It then
attempts to fire each possible address and loops once it gets a result, ensuring
that the address was assigned correctly (or at least showing what the current
address is if it was unsuccessful).

Adapted from code example by MaxBotix found here: <http://www.maxbotix.com/articles/028.htm#arduino>
and project found here: <https://www.arduino.cc/en/Tutorial/SFRRangerReader>

Sensor Connections:
Pin 7 to GND
Pin 6 to 5V
Pin 5 to SCL
Pin 4 to SDA
*/

#include "Wire.h"
// The Arduino Wire library uses the 7-bit version of the address
// so the code example uses 0x70 instead of the 8‑bit 0xE0
#define SensorAddress byte(0x70)
//The Sensor ranging command has a value of 0x51
#define RangeCommand byte(0x51)
//These are the two commands that need to be sent in sequence to change the sensor address
#define ChangeAddressCommand1 byte(0xAA)
#define ChangeAddressCommand2 byte(0xA5)

bool done = false;
byte testAddr = 0;
int reading = 0;

void setup() {
        Serial.begin(9600);//Open serial connection at 9600 baud
        Wire.begin();//Initiate Wire library for I2C communications with I2CXL‑MaxSonar‑EZ
}

void loop() {

        if (!done) {
          byte old = 112; // old address goes here (default it 112)
          changeAddress(old,0xe0,false);  // note: 2nd param will be shifted 1 bit left
          done = true;
        }
        else {
          Serial.print("Test Address: ");
          Serial.print(testAddr);
          Serial.print(" ");

          // step 1: instruct sensor to read
          Wire.beginTransmission(testAddr);
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
            Serial.println("SENSOR 1");
            Serial.print("Time: ");
            Serial.print(millis());
            Serial.print("\tRange: ");
            Serial.print(reading);   // print the reading
            Serial.println(" cm");
          }
          else {
            Serial.print("Sensor "); Serial.print(testAddr); Serial.println(" read error");
            // Increment testAddr until correct address is found
            testAddr++;
          }
        }

}

/* Commands a sensor at oldAddress to change its address to newAddress
oldAddress must be the 7-bit form of the address that is used by Wire
7BitHuh determines whether newAddress is given as the new 7 bit version or the 8 bit version of the address
\ If true, if is the 7 bit version, if false, it is the 8 bit version
*/
void changeAddress(byte oldAddress, byte newAddress, boolean SevenBitHuh){
       Wire.beginTransmission(oldAddress);                 //Begin addressing
       Wire.write(ChangeAddressCommand1);              //Send first change address command
       Wire.write(ChangeAddressCommand2);              //Send second change address command

       byte temp;
       if(SevenBitHuh){ temp = newAddress << 1; }     //The new address must be written to the sensor
       else     { temp = newAddress;         }               //in the 8bit form, so this handles automatic shifting
       Serial.println(newAddress);
       Serial.write(newAddress);
       //Serial.println();
       Wire.write(newAddress);                                          //Send the new address to change to
       Wire.endTransmission();
}
