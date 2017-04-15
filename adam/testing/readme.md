# Testing
This directory contains Arduino projects for testing sensors and MAVLink message transmission.

### Projects
* **freqmeasure_test**: Testing a single HB100 microwave sensor
* **mavlink_catcher**: Used for receiving and decoding specific MAVLink messages
* **mavlink_pitcher**: Sends MAVLink messages w/ hard-coded values
* **maxbotix_2test**: Alternately firing a pair of MaxBotix MB1242 sensors
* **maxbotix_6test**: Firing six MaxBotix MB1242 sensors in sequence
* **maxbotix_6test_doublefire**: Six MaxBotix sensors, firing two opposing sensors at a time
* **maxbotix_6test_doublefire_w_mavlink**: Adds MAVLink message transmission to maxbotix_6test_doublefire
* **maxbotix_changeAddress**: Used to change the I2C address of a MaxBotix MB1242 sensor (and/or check what the address of one is)
* **maxbotix_test**: Testing a single MaxBotix MB1242 sensor
* **ultrawave_test**: Testing a single MaxBotix/HB100 sensor set
* **ultrawave_2test**: Testing two MaxBotix/HB100 sensor sets (firing simultaneously)
* **ultrawave_6test**: Testing the full set of 6 MaxBotix/HB100 sensor sets
