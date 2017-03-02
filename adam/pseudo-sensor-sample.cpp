// Define time to perform an iteration of the loop (seconds)
#define ITER .05

int main()
  int s = 0;
  uint16_t dist, spd, raw, res;
  unsigned long t;
  unsigned long* lastFire = new unsigned long[6];
  
  while (true)  {
    // Make sure at least 1/10 sec has passed (max firing rate)
    t = millis();
    if ((t - lastFire[s]) > 100) {
      lastFire[s] = currentTime;
      // Fire ultrasonic
      dist = getUltrasonic(s);
      // Fire microwave
      raw = getMicrowave(s);
      // Translate analog microwave input into speed
      spd = rawToSpeed(raw);
            
      // Predict location @ next firing
      res = dist - spd*6*ITER;
      if (res <= 0) {
        res = .5;
      }
      
      // Generate MAVLINK message
      msg = makeMAV(t,res,s);
    }
    // Update which sensor to fire
    s++;
    s = s % 6;
  }
}
