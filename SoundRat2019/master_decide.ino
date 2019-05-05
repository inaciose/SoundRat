/*
 * 
 */

void bodyMoveDecide() {
  int i;
  int maxDistance = 0;
  byte maxDistanceIdx = 0;

  byte emptyIndex[READ_POSITION_MAX];
  byte emptyIndexIdx = 0;

  boolean needToGoBackward = false;
  float theta;

  #if !PLOTTER_OUTPUT
    Serial.print("READINGS"); Serial.println("\t");
  for(i = 0; i < READ_POSITION_MAX; i++) {
    Serial.print(readPosition[i]); Serial.print("\t");
  }
  Serial.println("\t");
  #endif
  for(i = 0; i < READ_POSITION_MAX; i++) {
    #if !PLOTTER_OUTPUT
      Serial.print(readPositionDistance[i]); Serial.print("\t");
    #endif
    // update max distance found
    if(readPositionDistance[i] > maxDistance) {
      maxDistance = readPositionDistance[i];
      maxDistanceIdx = i;
    }
    // update unknow distance array
    if(readPositionDistance[i] == 0) {
      emptyIndex[emptyIndexIdx] = i;
      emptyIndexIdx++;
    }
  }
  #if !PLOTTER_OUTPUT
    Serial.println("\t");
  #endif
  
  // check if we can rotate
  boolean canRotateLeft = true;
  boolean canRotateRight = true;

  if(((sonarValue[SONAR_LEFT_LEFT] > 0) && (sonarValue[SONAR_LEFT_LEFT] < WALL_SIDE_ROTATE_THRESHOLD)) || ((sonarValue[SONAR_RIGHT_RIGHT] > 0) && (sonarValue[SONAR_RIGHT_RIGHT] < WALL_SIDE_THRESHOLD))) {
    canRotateLeft = false;
  }
  if(((sonarValue[SONAR_RIGHT_RIGHT] > 0) && (sonarValue[SONAR_RIGHT_RIGHT] < WALL_SIDE_ROTATE_THRESHOLD)) || ((sonarValue[SONAR_LEFT_LEFT] > 0) && (sonarValue[SONAR_LEFT_LEFT] < WALL_SIDE_THRESHOLD))) {
    canRotateRight = false;
  }

  // we have done a pre selection of best possible directions
  // the ones we have more space lets do a final choose
  if(!emptyIndexIdx) {
    //
    // he have no unknow distance readings
    //
    if(!canRotateLeft && !canRotateRight && readPositionDistance[maxDistanceIdx] < WALL_FRONT_ALLEY_THRESHOLD) {
      // we are in a alley, and cant rotate.
      needToGoBackward = true;
      theta = 180;
      //selectedIdx = READ_POSITION_MAX;
      #if !PLOTTER_OUTPUT
        Serial.println("SELECT GoBackward ");
      #endif
    } else {
      if(readPositionDistance[maxDistanceIdx] < WALL_FRONT_ALLEY_THRESHOLD) {
        // we are in a alley and can rotate
        if(canRotateLeft) {
          // rotate 180 degrees left
          theta = 180;
          #if !PLOTTER_OUTPUT
            Serial.println("SELECT rotate 180");
          #endif
        } else {
          // rotate 180 degrees right
          theta = -180;
          #if !PLOTTER_OUTPUT
            Serial.println("SELECT rotate -180");
          #endif
        }
        //selectedIdx = READ_POSITION_MAX;      
      } else {
        // be stupid, select the max distance
        theta = readPosition[maxDistanceIdx] - 90;
        //selectedIdx = maxDistanceIdx;
        #if !PLOTTER_OUTPUT
          Serial.print("SELECT MAX ");
          Serial.print(readPosition[maxDistanceIdx]); Serial.print("\t");
          //Serial.print(readPositionDistance[maxDistanceIdx]); Serial.print("\t");
          Serial.print(theta); Serial.println("\t");
        #endif
      }
     
    }    
  } else {
    // we have unknow distance readings
    // must select one from the available
    // be stupid, select the first
    theta = readPosition[emptyIndex[0]] - 90;
    //selectedIdx = emptyIndex[0];
    #if !PLOTTER_OUTPUT
      Serial.print("SELECT empty ");
      Serial.print(readPosition[emptyIndex[0]]); Serial.print("\t");
      //Serial.print(readPositionDistance[0]); Serial.print("\t");
      Serial.print(theta); Serial.println("\t");
    #endif
  }

  // control vars
  // dismiss reaction to obstacle (not check is setted)
  frontObstacleDetectedStatus = 0;
  leftSideObstacleDetectedStatus = 0;
  rightSideObstacleDetectedStatus = 0;
  leftFrontSideObstacleDetectedStatus = 0;
  rightFrontSideObstacleDetectedStatus = 0;
  
  // end sequence
  resetPrgCmd();
  // load program
  if(needToGoBackward) {
    Serial.println("LOAD PROGRAM BACKWARD");
    prgCmd[0] = {CMD_GENERIC, 0, EXT_CMD_RUN, 0, -STD_RUN_NOWAY_DISTANCE, 0, 0, 0, 0, 0};
  } else {
    if(!theta) {
      // go front
      Serial.println("LOAD PROGRAM GO");
      prgCmd[0] = {CMD_GENERIC, 0, EXT_CMD_RUN, 0, STD_RUN_DISTANCE, 0, STD_TARGET_SPEED, 0, 0, 0};
    } else {
      // rotate and go
      Serial.println("LOAD PROGRAM ROTATE");
      prgCmd[0] = {CMD_GENERIC, 0, EXT_CMD_ROTATE, 0, theta, 0, 0, 0, 0, 0};
      prgCmd[1] = {CMD_GENERIC, 0, EXT_CMD_RUN, 0, STD_RUN_DISTANCE, 0, STD_TARGET_SPEED, 0, 0, 0};      
    }
  }
      
  Serial.println("LOAD PROGRAM");
  //int i;
  for(i=0; i < PRG_CMD_MAX; i++) {
    Serial.print(i); Serial.print("\t");
    Serial.print(prgCmd[i].cmd); Serial.print("\t");
    Serial.print(prgCmd[i].intCmd); Serial.print("\t");
    Serial.print(prgCmd[i].motionCmd); Serial.print("\t");
    Serial.print(prgCmd[i].motionChangeCmd); Serial.print("\t|\t");
    Serial.print(prgCmd[i].v1); Serial.print("\t");
    Serial.print(prgCmd[i].v2); Serial.print("\t");
    Serial.print(prgCmd[i].v3); Serial.print("\t");
    Serial.print(prgCmd[i].v4); Serial.print("\t");
    Serial.print(prgCmd[i].v5); Serial.print("\t");
    Serial.print(prgCmd[i].radius); Serial.println("\t");
  }
  Serial.print(prgIdx); Serial.println("");

}
