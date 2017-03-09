# ADAM Arduino Custom Software

### Functionality
* Fire sensors
* Receive sensor input
* Translate input
  * **HB100**: Translate raw input into frequency w/ FreqPeriod library, then translate into centimeters per second
    * **(Input frequency) * 1.42523 = speed in cm/s**
  * **[MaxSonar]**((http://www.maxbotix.com/documents/I2CXL-MaxSonar-EZ_Datasheet.pdf)): 2 byte distance (cm)
  * `result = dist - (spd * 6 * INTERVAL)`
    * INTERVAL is the time it takes to fire a given sensor set (i.e. microwave and ultrasonic facing the same direction)
    * Could use 100ms instead of 6*INTERVAL b/c that's max refresh rate for ultrasonic sensors (required for acoustic dissipation)
      * Could fire opposing ultrasonic sensors at smaller interval (i.e. sensors 180 degrees apart)
* Generate & transmit MAVLink message
  * Distance (result) must be uint16_t for MAVLink DISTANCE_SENSOR

### Contents
* **adam**: Final code to upload to Arduino
* **HB100_test**: Test project found in Resources (below)
* **maxbotix_test**: Test project for a single MaxBotix I2CXL MaxSonar EZ MB1242 sensor

### To do
* Develop firing sequence algorithm
* Determine MAVLink message generation/transmission method

## Resources
* [MAVLink how-to](http://discuss.ardupilot.org/t/mavlink-step-by-step/9629)
* [MAVLink on GitHub](https://github.com/mavlink/mavlink)
* [HB100 GitHub Example Project](https://github.com/3zuli/HB100_test)
* [Another HB100 project](https://www.gitbook.com/book/kd8bxp/arduino-project-doppler-radar-speed-detection-usi/details)
* [HB100 Preamp](https://hackaday.io/project/371-hb100-radar-shield)
* [Arduino I2C ultrasonic rangefinder tutorial](https://www.arduino.cc/en/Tutorial/SFRRangerReader)
* [FreqPeriod found here](http://interface.khm.de/index.php/lab/interfaces-advanced/frequency-measurement-library/)
*
