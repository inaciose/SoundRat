
// BOF: SONAR


void processSonar() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= sonarPingTimer[i]) {         // Is it this sensor's time to ping?
      sonarPingTimer[i] += SONAR_PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && sonarIndex == SONAR_NUM - 1) sonarOneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[sonarIndex].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      sonarIndex = i;                          // Sensor being accessed.
      sonarRead[sonarIndex] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      // filter zero anomalies on value
      if(sonarZeroCounter[sonarIndex]++ > SONAR_ZERO_MIN) {
        sonarValue[sonarIndex] = 0;
      }      
      sonar[sonarIndex].ping_timer(sonarEchoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }  
}


void sonarEchoCheck() {  
  // If ping received, set the sensor distance to array.
  if (sonar[sonarIndex].check_timer()) {
    sonarZeroCounter[sonarIndex] = 0;
    sonarRead[sonarIndex] = sonar[sonarIndex].ping_result / US_ROUNDTRIP_CM * SONAR_SCALE;

    // try to filter large discrepancies on current read compared with last read
    if(abs(sonarKalman[sonarIndex] - sonarRead[sonarIndex])  > sonarKalman[sonarIndex] * SONAR_MAX_ANOMALY / 100) {
      // there is an anomaly
      if(sonarAnomalyCounter[sonarIndex] < 1) { // HARDCODED
        // if bellow the limit use last stable value
        sonarRead[sonarIndex] = sonarKalman[sonarIndex];        
      }
      // increment counter to later trigger
      sonarAnomalyCounter[sonarIndex]++;
    } else {
      // there is no anomaly, reset counter
      sonarAnomalyCounter[sonarIndex] = 0;
    }

    // now after any previous adjusts to the input we calculate a stable value
    switch(sonarIndex) {
      case 0:
        sonarKalman[sonarIndex] = sonarKalman0.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 1:
        sonarKalman[sonarIndex] = sonarKalman1.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 2:
        sonarKalman[sonarIndex] = sonarKalman2.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 3:
        sonarKalman[sonarIndex] = sonarKalman3.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 4:
        sonarKalman[sonarIndex] = sonarKalman4.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 5:
        sonarKalman[sonarIndex] = sonarKalman5.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
    }
    // set current read value for stable
    sonarValue[sonarIndex] = sonarKalman[sonarIndex];
    // reset zero range counter
    sonarZeroCounter[sonarIndex] = 0;
  }
}

/*
void processSonarOld() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= sonarPingTimer[i]) {         // Is it this sensor's time to ping?
      sonarPingTimer[i] += SONAR_PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && sonarIndex == SONAR_NUM - 1) sonarOneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[sonarIndex].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      sonarIndex = i;                          // Sensor being accessed.
      sonarRead[sonarIndex] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[sonarIndex].ping_timer(sonarEchoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }  
}



void sonarEchoCheckOld() {  
  // If ping received, set the sensor distance to array.
  if (sonar[sonarIndex].check_timer()) {
    sonarZeroCounter[sonarIndex] = 0;
    sonarRead[sonarIndex] = sonar[sonarIndex].ping_result / US_ROUNDTRIP_CM * SONAR_SCALE;
    
    // try to filter large discrepancies on current read compared with last read
    if(abs(sonarKalman[sonarIndex] - sonarRead[sonarIndex])  > sonarKalman[sonarIndex] * SONAR_MAX_ANOMALY / 100) {
      // there is an anomaly
      if(sonarAnomalyCounter[sonarIndex] < 1) { // HARDCODED
        // if bellow the limit use last stable value
        sonarRead[sonarIndex] = sonarKalman[sonarIndex];        
      }
      // increment counter to later trigger
      sonarAnomalyCounter[sonarIndex]++;
    } else {
      // there is no anomaly, reset counter
      sonarAnomalyCounter[sonarIndex] = 0;
    }

    // now after any previous adjusts to the input we calculate a stable value
    switch(sonarIndex) {
      case 0:
        sonarKalman[sonarIndex] = sonarKalman0.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 1:
        sonarKalman[sonarIndex] = sonarKalman1.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 2:
        sonarKalman[sonarIndex] = sonarKalman2.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
      case 3:
        sonarKalman[sonarIndex] = sonarKalman3.getFilteredValue(sonarRead[sonarIndex]); // kalman  
        break;
    }
    // set current read value for stable
    sonarValue[sonarIndex] = sonarKalman[sonarIndex];
  } else {
    // we didnt receive any ping in sonar.check_timer() 
    // so we need to manage if the value is 0 or the stable reading
    if(sonarZeroCounter[sonarIndex] > SONAR_ZERO_MIN) {
      // we didnot receive a ping for enougth time to be zero
      sonarValue[sonarIndex] = 0;
    } else {
      // still wait we need to use last stable
      sonarValue[sonarIndex] = sonarKalman[sonarIndex];
    }
    // control the value 0
    sonarZeroCounter[sonarIndex]++;
    if(sonarZeroCounter[sonarIndex] > 65530) sonarZeroCounter[sonarIndex] = SONAR_ZERO_MIN + 1;
  }
}
*/

void sonarOneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  // do nothing... its displayed on main loop
  // must keep the function
}
void setupSonar() {
  delay(50);
  int i = 0, j;
  while(i < 10) {
    for(j=0; j < SONAR_NUM; j++) {
      sonarRead[j] = sonar[j].ping_cm() * SONAR_SCALE;
      if(sonarRead[j] != 0) {
        switch(j) {
          case 0: 
            sonarKalman[j] = sonarKalman0.getFilteredValue(sonarRead[j]);
            sonarValue[j] = sonarKalman0.getFilteredValue(sonarRead[j]);
            break;
          case 1: 
            sonarKalman[j] = sonarKalman1.getFilteredValue(sonarRead[j]);
            sonarValue[j] = sonarKalman1.getFilteredValue(sonarRead[j]);
            break;
          case 2:
            sonarKalman[j] = sonarKalman2.getFilteredValue(sonarRead[j]);
            sonarValue[j] = sonarKalman2.getFilteredValue(sonarRead[j]);
            break;
          case 3:
            sonarKalman[j] = sonarKalman3.getFilteredValue(sonarRead[j]);
            sonarValue[j] = sonarKalman3.getFilteredValue(sonarRead[j]);
            break;
        }
      }
      delay(33);
    }
    i++;
  }

  sonarPingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    sonarPingTimer[i] = sonarPingTimer[i - 1] + SONAR_PING_INTERVAL;
}
// EOF: SONAR
