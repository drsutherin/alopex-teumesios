// How long it takes to perform an iteration of the loop (seconds)
#DEFINE REFRESH .03

int main()
  Time* lastFireTime = new Time[6];
  Sensor* sensor = new Sensor[6];

  while (true)  {
    // Fire 6 sensors in sequence
    sensor = sensor % 6;
    // Make sure at least 1/10 sec has passed (max firing rate for sensors)
    if ((currentTime - lastFireTime[sensor]) > .1) {
      lastFireTime[sensor] = currentTime;
      // Fire ultrasonic
      dist = getUltrasonic[sensor];
      // Fire microwave
      spd = getMicrowave[sensor];
      // Predict location @ next firing
      //  (asssuming next fire @ currentTime+6*REFRESH)
      result = dist - spd*6*REFRESH;
      sensor++;
    }
  }
}
