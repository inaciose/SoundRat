/*
 * 
 */

void displayLoopData() {
/*
  Serial.print(leftSideObstacleDetectedStatus); Serial.print("\t");
  Serial.print(leftFrontSideObstacleDetectedStatus); Serial.print("\t");
  Serial.print(frontObstacleDetectedStatus); Serial.print("\t");
  Serial.print(rightSideObstacleDetectedStatus); Serial.print("\t");
  Serial.print(rightFrontSideObstacleDetectedStatus); Serial.print("\t|\t");
  //Serial.print(prgIdx); Serial.print("\t");
  Serial.print(theta*RAD_TO_DEG); Serial.print("\t");
  Serial.print(px); Serial.print("\t");
  Serial.print(py); Serial.print("\t");
  Serial.print(wl); Serial.print("\t");
  Serial.print(wr); Serial.print("\t");
  Serial.print(pd); Serial.print("\t");
*/  
  Serial.print(prgCmd[prgIdx].motionCmd); Serial.print("\t");
  Serial.print(prgCmd[prgIdx].motionChangeCmd); Serial.print("\t|\t");
  

  //Serial.print(frontServoAngle); Serial.print("\t");  
  //Serial.print(frontLidarRead / 10); Serial.print("\t");
  
  Serial.print(sonarValue[0] / SONAR_SCALE); Serial.print("\t");
  Serial.print(sonarValue[2] / SONAR_SCALE); Serial.print("\t");
  Serial.print(sonarValue[5] / SONAR_SCALE); Serial.print("\t");
  Serial.print(sonarValue[3] / SONAR_SCALE); Serial.print("\t");
  Serial.print(sonarValue[1] / SONAR_SCALE); Serial.print("\t");
  Serial.print(sonarValue[4] / SONAR_SCALE); Serial.print("\t|\t");
Serial.println();
/*
 
  displayMotorInfo();
  
  Serial.print(motorData[5].b[0]); Serial.print("\t"); // loopTime
  Serial.print(loopTime); Serial.print("\t ");
  Serial.println();
*/
  return;

  // only display when any value change
  bool disp = false;
  static int lastValue[SONAR_NUM];
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if(abs(lastValue[i] - sonarValue[i]) > 10 ) {
      lastValue[i] = sonarValue[i];
      disp = true;
    }
  }

  // have changed? display!
  if(disp) {
    Serial.print(sonarValue[0] / SONAR_SCALE); Serial.print("\t");
    Serial.print(sonarValue[2] / SONAR_SCALE); Serial.print("\t");
    Serial.print(sonarValue[5] / SONAR_SCALE); Serial.print("\t");
    Serial.print(sonarValue[3] / SONAR_SCALE); Serial.print("\t");
    Serial.print(sonarValue[1] / SONAR_SCALE); Serial.print("\t");
    Serial.print(sonarValue[4] / SONAR_SCALE); Serial.print("\t");
    Serial.println();
  }
 
///*
  Serial.print(frontObstacleDetectedStatus); Serial.print("\t");
  Serial.print(leftSideObstacleDetectedStatus); Serial.print("\t");
  Serial.print(rightSideObstacleDetectedStatus); Serial.print("\t|\t");
  //Serial.print(prgIdx); Serial.print("\t");
  Serial.print(prgCmd[prgIdx].motionCmd); Serial.print("\t");
  Serial.print(prgCmd[prgIdx].motionChangeCmd); Serial.print("\t|\t");
  

  displayMotorInfo();

  //for (uint8_t i = 0; i < SONAR_NUM; i++) {
    //Serial.print(sonarRead[i] / SONAR_SCALE); Serial.print("\t");
    //Serial.print(sonarKalman[i] / SONAR_SCALE); Serial.print("\t");
    //Serial.print(sonarValue[i] / SONAR_SCALE); Serial.print("\t");
  //}  

  Serial.print(loopTime); Serial.print("\t ");
  Serial.println();
  
//*/
  
}

void displayMotorInfo() {
  Serial.print(motorData[0].sc[0]); Serial.print("\t"); // motionCurrentDirection
  Serial.print(motorData[0].b[1]); Serial.print("\t"); // reportCmdReceivedStatus
  //Serial.print(motorData[0].b[2]); Serial.print("\t"); // mCmd.id
  Serial.print(motorData[0].b[3]); Serial.print("\t"); // mCmd.cmd
  Serial.print(motorData[1].b[0]); Serial.print("\t|\t"); // reportTriggerStatus
  /*
  Serial.print(motorData[1].b[1]); Serial.print("\t"); // useSingleSpeedPid (if not dual only)
  Serial.print(motorData[1].b[2]); Serial.print("\t"); // useSteeringPid
  Serial.print(motorData[1].sc[3]); Serial.print("\t"); // steeringPidInputLast
  Serial.print(motorData[2].sc[0]); Serial.print("\t"); // speedPidSetPoint - speedPidInputLast // leftSpeedPidSetPoint - leftSpeedPidInputLast
  Serial.print(motorData[2].sc[1]); Serial.print("\t"); // rightSpeedPidSetPoint - rightSpeedPidInputLast
  Serial.print(motorData[2].b[2]); Serial.print("\t"); // speedPidSetPoint // leftSpeedPidSetPoint
  Serial.print(motorData[2].b[3]); Serial.print("\t"); // rightSpeedPidSetPoint
  Serial.print(motorData[3].l); Serial.print("\t"); // encoderLeftTotalPulses
  Serial.print(motorData[4].l); Serial.print("\t"); // encoderRightTotalPulses
  Serial.print(motorData[5].b[0]); Serial.print("\t"); // loopTime
  */
}

  /*
  static int maxLoopTime = 0;
  static int minLoopTime = 1000;
  static long loopCount = 0;
  static unsigned long loopTotalTime = 0;
  loopCount ++;
  loopTotalTime += loopTime;
  float averageLoopTime = (float)loopTotalTime / (float)loopCount;
  if(loopTime < minLoopTime) minLoopTime = loopTime;
  if(loopTime > maxLoopTime) maxLoopTime = loopTime;
  Serial.print(averageLoopTime); Serial.print("\t");
  Serial.print(minLoopTime); Serial.print("\t");
  //Serial.print(loopTime); Serial.print("\t");
  Serial.print(maxLoopTime); Serial.println("\t");
  return;
  */

  
/*  
  // display info 
  bool disp = false;
  static int lastValue[SONAR_NUM];

  // only display when any value change
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if(abs(lastValue[i] - sonarValue[i]) > 10 ) {
      lastValue[i] = sonarValue[i];
      disp = true;
    }
  }

  // have changed? display!
  if(disp) {
    for (uint8_t i = 0; i < SONAR_NUM; i++) {
      //Serial.print(sonarRead[i] / SONAR_SCALE); Serial.print("\t");
      //Serial.print(sonarKalman[i] / SONAR_SCALE); Serial.print("\t");
      Serial.print(sonarValue[i] / SONAR_SCALE); Serial.print("\t");
    }  
    //Serial.print(serialCounter); Serial.print("\t ");
    //Serial.print(rec); Serial.print("\t");
    Serial.print(loopTime); Serial.print("\t ");
    Serial.println();
  }
*/  



float pulsesToUnits(signed long pulses) {
  return ((float)(pulses * bodyEncoderPulsesPerUnit));
}

float getDistanceDone(signed long pulsesLeft, signed long pulsesRight) {
  float tmp1 = pulsesLeft;
  float tmp2 = pulsesRight;
  // strange compensation to rotate right distance
  if(prgCmd[prgIdx].motionCmd == EXT_CMD_ROTATE) {
    if(prgCmd[prgIdx].v1 < 0) {
      tmp1 = pulsesLeft / 1.0095;
      tmp2 = pulsesRight / 1.0095;
    }
  }
  tmp1 = abs(pulsesToUnits(tmp1));
  tmp2 = abs(pulsesToUnits(tmp2));
  float ret = (tmp1 + tmp2) / 2;
  
  if(prgCmd[prgIdx].v1 < 0) {
    ret = -ret;
  }
  return ret;
}
