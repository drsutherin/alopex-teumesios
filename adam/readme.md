# ADAM Arduino Custom Software
* Fire sensors
* Receive sensor input
* Translate input
  * **HB100**: raw input -> kph -> centimeters = 100*(kph)^-1 -> uint16_t for MAVLink DISTANCE_SENSOR
    * NOTE: This is **not** the genuine distance of the object.  It's a spoof that allows us to make ArduPilot think that faster-moving objects are closer, regardless of their distance
    * Think about skipping FreqMeasure translation into kph
    * Instead, jump straight from raw input to "distance" in centimeters
  * **MaxSonar**: 2 byte distance (cm) -> uint16_t for MAVLink DISTANCE_SENSOR
    * Super "easy" (fingers crossed)

## General Ideas
* Avoid function calls as much as possible (too slow)
* 
