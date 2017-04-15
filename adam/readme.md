# ADAM Arduino Custom Software
**Autonomous Detection and Avoidance Module**: Used to poll the sensors, translate sensor input, and predict nearby object's future location based on current distance and speed.

### Functionality
##### Fires sensors
  * [MaxBotix MB1242 I2CXL MaxSonar EZ ultrasonic sensors](http://www.maxbotix.com/documents/I2CXL-MaxSonar-EZ_Datasheet.pdf)
  * [Singapore Technologies Electronics HB100 microwave sensors](https://www.limpkin.fr/public/HB100/HB100_Microwave_Sensor_Module_Datasheet.pdf)
##### Receives sensor input
  * HB100: analog frequency
  * MaxSonar: digital input as 2 byte distance (cm)

##### Translates input
  *  HB100: Use FreqMeasure library to get multiple frequency readings, then translate into centimeters per second
    * `input_frequency * 1.42523 = speed // in cm/s`
  * Use both readings to predict object's location the next time the sensor is fired
    * `result = dist - (spd * 3 * INTERVAL)`
    * INTERVAL is the time it takes to fire a given sensor set (i.e. microwave and ultrasonic facing the same direction)
    * 6 sensors sets are used, but `spd*INTERVAL*3` is used because opposing sensor sets are fired simultaneously
    * Speed value is only magnitude, not direction, so result is assumed worst-case given object is traveling directly at the sensor (i.e. the UAS)

##### Generates & transmits MAVLink messages
  * Sends DISTANCE_SENSOR messages via serial port

### Contents
* **adam**: Final code to upload to Arduino
* **testing**: Test code written during the development process for sensors & MAVLink message transmission/receipt

## Resources
* [MAVLink how-to](http://discuss.ardupilot.org/t/mavlink-step-by-step/9629)
* [MAVLink on GitHub](https://github.com/mavlink/mavlink)
* [HB100 GitHub Example Project](https://github.com/3zuli/HB100_test)
* [Another HB100 project](https://www.gitbook.com/book/kd8bxp/arduino-project-doppler-radar-speed-detection-usi/details)
* [HB100 Preamp](https://hackaday.io/project/371-hb100-radar-shield)
* [Arduino I2C ultrasonic rangefinder tutorial](https://www.arduino.cc/en/Tutorial/SFRRangerReader)
