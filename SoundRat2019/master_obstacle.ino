/*
 * 
 */

 
void obstacleDetection() {
  boolean frontObstacleDetected = false;
  boolean leftSideObstacleDetected = false;
  boolean rightSideObstacleDetected = false;
  boolean leftFrontSideObstacleDetected = false;
  boolean rightFrontSideObstacleDetected = false;
  static byte frontLowRangeCounter = 0;
  static byte leftSideLowRangeCounter = 0;
  static byte rightSideLowRangeCounter = 0;
  static byte leftFrontSideLowRangeCounter = 0;
  static byte rightFrontSideLowRangeCounter = 0;

  if(prgCmd[prgIdx].motionCmd == EXT_CMD_RUN || prgCmd[prgIdx].motionCmd == EXT_CMD_CURVE) {
    
    //
    // FRONT DETECTION
    //
    if(!frontObstacleDetectedStatus) {
      // check front distances
      if(sonarValue[SONAR_FRONT_LEFT] > 0 && sonarValue[SONAR_FRONT_LEFT] < WALL_FRONT_THRESHOLD) {
        if(!frontObstacleDetectedStatus) frontObstacleDetected = true;
      }  
      if(sonarValue[SONAR_FRONT_RIGHT] > 0 && sonarValue[SONAR_FRONT_RIGHT] < WALL_FRONT_THRESHOLD) {
        frontObstacleDetected = true;
      }
    }

    // filter front detection before set obstacle state
    // TODO: HARDCODED >= 0 not active
    if(frontObstacleDetected) { 
      if(frontLowRangeCounter >= 0) {
        if(!frontObstacleDetectedStatus) {
          frontObstacleDetectedStatus = 1;          
          #if !PLOTTER_OUTPUT
            Serial.print("FRONT OBSTACLE DETECTED: "); 
            Serial.print(sonarValue[SONAR_FRONT_LEFT]); Serial.print("\t"); 
            Serial.print(sonarValue[SONAR_FRONT_RIGHT]); Serial.println("\t");
          #endif  
        }
      }
      frontLowRangeCounter++;
    } else {
      frontLowRangeCounter = 0;
      //frontObstacleDetectedStatus = 0;
    }
    
    //
    // LEFT DETECTION
    //
    // LEFT LEFT
    if(!leftSideObstacleDetectedStatus) {
      // check left side distances
      if(sonarValue[SONAR_LEFT_LEFT] > 0 && sonarValue[SONAR_LEFT_LEFT] < WALL_SIDE_THRESHOLD) {
        leftSideObstacleDetected = true;
      }
    }

    // filter front detection before set obstacle state
    // TODO: HARDCODED >= 0 not active
    if(leftSideObstacleDetected) {
      if(leftSideLowRangeCounter >= 0) {
        leftSideObstacleDetectedStatus = 1;
        #if !PLOTTER_OUTPUT
          Serial.print("LEFT OBSTACLE DETECTED: "); 
          Serial.print(sonarValue[SONAR_LEFT_LEFT]); Serial.print("\t"); 
          Serial.print(sonarValue[SONAR_LEFT_FRONT]); Serial.println("\t");
        #endif  
      }
      leftSideLowRangeCounter++;
    } else {
      leftSideLowRangeCounter = 0;
      //leftSideObstacleDetectedStatus = 0;
    }
    
    // LEFT FRONT
    if(!leftFrontSideObstacleDetectedStatus) {
      // check left side distances
      if(sonarValue[SONAR_LEFT_FRONT] > 0 && sonarValue[SONAR_LEFT_FRONT] < WALL_SIDE_FRONT_THRESHOLD) {
        leftFrontSideObstacleDetected = true;
      }
    }

    // filter front detection before set obstacle state
    // TODO: HARDCODED >= 0 not active
    if(leftFrontSideObstacleDetected) {
      if(leftFrontSideLowRangeCounter >= 0) {
        leftFrontSideObstacleDetectedStatus = 1;
        #if !PLOTTER_OUTPUT
          Serial.print("LEFT FRONT OBSTACLE DETECTED: "); 
          Serial.print(sonarValue[SONAR_LEFT_LEFT]); Serial.print("\t"); 
          Serial.print(sonarValue[SONAR_LEFT_FRONT]); Serial.println("\t");
        #endif  
      }
      leftFrontSideLowRangeCounter++;
    } else {
      leftFrontSideLowRangeCounter = 0;
      //leftFrontSideObstacleDetectedStatus = 0;
    }


    //
    // RIGHT DETECTION
    //
    // RIGHT FRONT
    if(!rightFrontSideObstacleDetectedStatus) {      
      if(sonarValue[SONAR_RIGHT_FRONT] > 0 && sonarValue[SONAR_RIGHT_FRONT] < WALL_SIDE_FRONT_THRESHOLD) {
        rightFrontSideObstacleDetected = true;
      }
    }

    // filter front detection before set obstacle state
    // TODO: HARDCODED >= 0 not active
    if(rightFrontSideObstacleDetected) {
      if(rightFrontSideLowRangeCounter >= 0) {
        rightFrontSideObstacleDetectedStatus = 1;
        #if !PLOTTER_OUTPUT
          Serial.print("RIGHT FRONT OBSTACLE DETECTED: "); 
          Serial.print(sonarValue[SONAR_RIGHT_FRONT]); Serial.print("\t"); 
          Serial.print(sonarValue[SONAR_RIGHT_RIGHT]); Serial.println("\t");
        #endif  
      }
      rightFrontSideLowRangeCounter++;
    } else {
      rightFrontSideLowRangeCounter = 0;
      //rightFrontSideObstacleDetectedStatus = 0;
    }

    // RIGHT RIGHT
    if(!rightSideObstacleDetectedStatus) {      
      if(sonarValue[SONAR_RIGHT_RIGHT] > 0 && sonarValue[SONAR_RIGHT_RIGHT] < WALL_SIDE_THRESHOLD) {
        rightSideObstacleDetected = true;
      }
    }

    // filter front detection before set obstacle state
    // TODO: HARDCODED >= 0 not active
    if(rightSideObstacleDetected) {
      if(rightSideLowRangeCounter >= 0) {
        rightSideObstacleDetectedStatus = 1;
        #if !PLOTTER_OUTPUT
          Serial.print("RIGHT OBSTACLE DETECTED: "); 
          Serial.print(sonarValue[SONAR_RIGHT_FRONT]); Serial.print("\t"); 
          Serial.print(sonarValue[SONAR_RIGHT_RIGHT]); Serial.println("\t");
        #endif  
      }
      rightSideLowRangeCounter++;
    } else {
      rightSideLowRangeCounter = 0;
      //sideObstacleDetectedStatus = 0;
    }
  }
}
