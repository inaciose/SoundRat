/*
 * 
 */

void intCmdProcess() {
  //Serial.print("intCmdProcess "); Serial.println("\t");
  
  // internal commands while slave is stoped
  if(intCmdStatus && !motionCmdWaitingId) {
    switch(prgCmd[prgIdx].intCmd) {
      case INT_CMD_SCAN_FRONT: // front readings 
        // TODO: change to servo sweep or a lidar
        // for now use some sonar readings
        // initialize all for a very low value
        for(int f = 0; f < READ_POSITION_MAX - 1; f++) readPositionDistance[f] = 1;
        // set the ones we have readings
        // 0 degrees
        readPositionDistance[0] = sonarValue[SONAR_RIGHT_RIGHT];
        // 45 degrees
        readPositionDistance[(READ_POSITION_MAX / 2) - (READ_POSITION_MAX / 4)] = sonarValue[SONAR_RIGHT_FRONT];
        // 75 degrees
        readPositionDistance[((READ_POSITION_MAX - 1) / 2) - 1] = sonarValue[SONAR_FRONT_LEFT];
        // 90 degrees
        // TODO: must assure that its not a false reflection
        if(sonarValue[SONAR_FRONT_LEFT] > WALL_FRONT_THRESHOLD && sonarValue[SONAR_FRONT_RIGHT] > WALL_FRONT_THRESHOLD) { // HARDCODED
          readPositionDistance[(READ_POSITION_MAX - 1) / 2] = (sonarValue[SONAR_FRONT_LEFT] + sonarValue[SONAR_FRONT_RIGHT]) / 2;
        }
        // 105 degrees
        readPositionDistance[((READ_POSITION_MAX - 1) / 2) + 1] = sonarValue[SONAR_FRONT_RIGHT];
        // 135 degrees
        readPositionDistance[(READ_POSITION_MAX / 2) + (READ_POSITION_MAX / 4)] = sonarValue[SONAR_LEFT_FRONT];
        // 180 degrees
        readPositionDistance[READ_POSITION_MAX - 1] = sonarValue[SONAR_LEFT_LEFT];
        // its done
        intCmdStatus = 0;
        cmdEnd();
        break;
      case INT_CMD_DECIDE: // decide
        bodyMoveDecide();
        intCmdStatus = 0;
        cmdStatus = 0;
        break;
      case INT_CMD_GOTO:      // goto prgIdx command
      case INT_CMD_PAUSE:     // pause the program counter (keep executing pause)
      case INT_CMD_RESTART:   // restart the program counter (keep program)
        //Serial.println("GO END ");
        intCmdStatus = 0;
        cmdEnd();
        break;
    }
  }
  // internal commands while slave is running
  //if(intCmdStatus && motionCmdWaitingId) {}
  
}

void motionCmdProcess () {
  if(!motionCmdStatus) {
    //
    // process sent stop terminated commands
    //
    if(intStopCmdStatus || extStopCmdStatus) {
      #if !PLOTTER_OUTPUT
        Serial.println("STOP CONFIRMED");
      #endif
      // log interrupted program TODO: check and correct?
      // calculate distance done using: encoderLeftTotalPulses, encoderRightTotalPulses
      //float distanceDone = getDistanceDone(motorData[0][3].l, motorData[0][4].l); 
      //stopedExtCmdUnits = stopedExtCmdUnits - distanceDone;
      //logCmd(stopedExtCmd, targetUnits, stopedExtCmdUnits);
      // reset storage
      //stopedExtCmdUnits = 0;
      //stopedExtCmd = 0;
      // end sequence
      resetPrgCmd();
      //Serial.print(frontObstacleDetectedStatus); Serial.println("");

      if(extStopCmdStatus) {
        // EXTERNAL STOP
        // load program
        prgCmd[0] = {CMD_GENERIC, INT_CMD_PAUSE, 0, 0, 0, 0, 0, 0, 0, 0};
        // reset control
        extStopCmdStatus = false;
      } else {
        // INTERNAL STOP
        // we stop because there is an obstacle in front?
        if(frontObstacleDetectedStatus || leftFrontSideObstacleDetectedStatus || rightFrontSideObstacleDetectedStatus) {
          // load program
          prgCmd[0] = {CMD_GENERIC, 0, EXT_CMD_RUN, 0, -50, 0, 0, 0, 0, 0};
          prgCmd[1] = {CMD_GENERIC, INT_CMD_SCAN_FRONT, 0, 0, 0, 0, 0, 0, 0, 0};
          prgCmd[2] = {CMD_GENERIC, INT_CMD_DECIDE, 0, 0, 0, 0, 0, 0, 0, 0};
          /*
          int i;
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
          */
        }
        // reset control
        intStopCmdStatus = false;
      }
    }
  }  
}

void cmdProcess() {
  //
  // run on all loops
  //

  //
  // IDLE CONTROL
  //
  if(!cmdStatus) {
    idleLoopCounter++;
    // we are idle for some time, lets run?
    if(idleLoopCounter > IDLE_LOOP_COUNTER_MAX) { 
      // just go front
      // TODO: a smarter behavior
      // load program
      prgCmd[0] = {CMD_GENERIC, 0, EXT_CMD_RUN, 0,STD_RUN_DISTANCE, 0, STD_TARGET_SPEED, 0, 0, 0};
    }
  } else {
    idleLoopCounter = 0;
  }
  //Serial.println("cmdProcess() idle control done");

  //
  // RECEIVED COMMAND (PROCESS STEP 1)
  //
/*
  if(piCmd.cmd) {
    switch(piCmd.cmd) {
      //
      // RECEIVED STOP
      //
      case REC_CMD_STOP_NOW:
        switch(prgCmd[prgIdx].motionCmd) {
          case EXT_CMD_RUN:
          case EXT_CMD_ROTATE:
          case EXT_CMD_CURVE:
            #if !PLOTTER_OUTPUT
              Serial.println("STOP REQUEST MOTOR SEND STOP");    
            #endif
            // store current extCmd data
            //stopedExtCmd = prgCmd[prgIdx].motionCmd;
            //stopedExtCmdUnits = prgCmd[prgIdx].v1;
            // update current command
            prgCmd[prgIdx].motionChangeCmd = EXT_CMD_STOP;
            // send stop command
            sendMotorSlaveMotionCmd(EXT_CMD_STOP, 0, 0, 0, 0, 0);
            // reset control      
            piCmd.cmd = 0;
            // set control variable    
            extStopCmdStatus = true;
          break;
        }
        break;
      //
      // RECEIVED RUN
      //
      case REC_CMD_RUN_NOW: 
        switch(prgCmd[prgIdx].intCmd) {
          case INT_CMD_PAUSE:
          // end sequence
          resetPrgCmd();
          // load program
          prgCmd[0] = {CMD_GENERIC, 0, EXT_CMD_RUN, 0,STD_RUN_DISTANCE, 0, STD_TARGET_SPEED, 0, 0, 0};
          // reset control      
          piCmd.cmd = 0;
          // allow new command processing
          motionCmdStatus = 0;
          break;
        }
        break;
      //
      // OTHER CMD
      //
       default:
        // reset control      
        piCmd.cmd = 0;
        break;
    }
  }
*/
  //
  // OBSTACLE DETECTED (PROCESS STEP 1)
  //
  // Only handle the stop if we are not waiting to stop
  if(!intStopCmdStatus) {
    if(frontObstacleDetectedStatus == 1) {
      //
      // FRONT OBSTACLE
      //
      #if !PLOTTER_OUTPUT
        Serial.println("FRONT OBSTACLE MOTOR SEND STOP");    
      #endif
      // store current extCmd data
      //stopedExtCmd = prgCmd[prgIdx].motionCmd;
      //stopedExtCmdUnits = prgCmd[prgIdx].v1;
      // update current command
      prgCmd[prgIdx].motionChangeCmd = EXT_CMD_STOP;
      // send stop command
      sendMotorSlaveMotionCmd(EXT_CMD_STOP, 0, 0, 0, 0, 0);
      // set control variable    
      intStopCmdStatus = true;
      frontObstacleDetectedStatus = 2;    
    } else if(leftSideObstacleDetectedStatus == 1) {
      //
      // LEFT SIDE OBSTACLE
      //
      #if !PLOTTER_OUTPUT
        Serial.println("LEFT OBSTACLE MOTOR SEND CURVE");
      #endif
  
      // update current command
      prgCmd[prgIdx].motionChangeCmd = EXT_CMD_CURVE;
      // send right curve command
      sendMotorSlaveCurveCmd(EXT_CMD_CURVE, -STD_SIDE_CURVE_ANGLE, STD_SIDE_CURVE_RADIUS, 0);
  
      // set control
      leftSideObstacleDetectedStatus = 2;
    } else if( rightSideObstacleDetectedStatus == 1) {
      //
      // RIGHT SIDE OBSTACLE
      //
      #if !PLOTTER_OUTPUT
        Serial.println("RIGHT OBSTACLE MOTOR SEND CURVE");
      #endif
  
      // update current command
      prgCmd[prgIdx].motionChangeCmd = EXT_CMD_CURVE;
      // send left curve command
      sendMotorSlaveCurveCmd(EXT_CMD_CURVE, STD_SIDE_CURVE_ANGLE, STD_SIDE_CURVE_RADIUS, 0);
  
      // set control
      rightSideObstacleDetectedStatus = 2;
    } else if(leftFrontSideObstacleDetectedStatus == 1) {
      //
      // LEFT FRONT SIDE OBSTACLE
      //
      #if !PLOTTER_OUTPUT
        Serial.println("LEFT FRONT OBSTACLE MOTOR SEND CURVE");
      #endif
  
      // store current extCmd data
      //stopedExtCmd = prgCmd[prgIdx].motionCmd;
      //stopedExtCmdUnits = prgCmd[prgIdx].v1;
      // update current command
      prgCmd[prgIdx].motionChangeCmd = EXT_CMD_STOP;
      // send stop command
      sendMotorSlaveMotionCmd(EXT_CMD_STOP, 0, 0, 0, 0, 0);
      // set control variable    
      intStopCmdStatus = true;
  
      // update current command
      //prgCmd[prgIdx].motionChangeCmd = EXT_CMD_CURVE;
      // send right curve command
      //sendMotorSlaveCurveCmd(EXT_CMD_CURVE, -STD_SIDE_CURVE_ANGLE, -STD_SIDE_CURVE_RADIUS, -STD_SIDE_OBSTACLE_SPEED_DIF);
  
      // set control
      leftFrontSideObstacleDetectedStatus = 2;
      
    } else if(rightFrontSideObstacleDetectedStatus == 1) {
      //
      // RIGHT FRONT SIDE OBSTACLE
      //
      #if !PLOTTER_OUTPUT
        Serial.println("RIGHT FRONT OBSTACLE MOTOR SEND CURVE");
      #endif
  
      // store current extCmd data
      //stopedExtCmd = prgCmd[prgIdx].motionCmd;
      //stopedExtCmdUnits = prgCmd[prgIdx].v1;
      // update current command
      prgCmd[prgIdx].motionChangeCmd = EXT_CMD_STOP;
      // send stop command
      sendMotorSlaveMotionCmd(EXT_CMD_STOP, 0, 0, 0, 0, 0);
      // set control variable    
      intStopCmdStatus = true;
  
      // update current command
      //prgCmd[prgIdx].motionChangeCmd = EXT_CMD_CURVE;
      // send left curve command
      //sendMotorSlaveCurveCmd(EXT_CMD_CURVE, STD_SIDE_CURVE_ANGLE, -STD_SIDE_CURVE_RADIUS, -STD_SIDE_OBSTACLE_SPEED_DIF);
  
      // set control
      rightFrontSideObstacleDetectedStatus = 2;
    }
  }
  //Serial.println("cmdProcess() OBSTACLE DETECTED (PROCESS STEP 1) done");
  
  intCmdProcess();
  //Serial.println("cmdProcess() intCmdProcess() done");
  motionCmdProcess();
  //Serial.println("cmdProcess() motionCmdProcess() done");

  if(!cmdStatus) {
    //Serial.println("cmdProcess() !cmdStatus begin");
    cmdStart();
    //Serial.println("cmdProcess() !cmdStatus done");
  } else {
    //
    // run on all the others
    //
    //Serial.println("cmdProcess() cmdStatus begin");
    // update motor slave data
    getMotorSlaveUpdate();
    //Serial.println("cmdProcess() cmdStatus getMotorSlaveUpdate() done");
    // detect trigger end command
    detectMotionChange();
    //Serial.println("cmdProcess() cmdStatus detectMotionChange() done");
    // detect command reception reported by slave
    detectCmdReception();
    //Serial.println("cmdProcess() cmdStatus detectCmdReception() done");
    // detect motion end reported by slave
    detectMotionEnd();
    //Serial.println("cmdProcess() cmdStatus detectMotionEnd() done");
  }

  //Serial.println("cmdProcess() all done");
}

void detectMotionChange() {
    // we are waiting for command end trigger and it arrives?
  if(prgCmd[prgIdx].motionChangeCmd && motorData[1].b[0] && motionCmdWaitingId && !motionCmdRequest) { // reportTriggerStatus
    // set flag for odometry
    lastMoveData = true;
    
    if(prgCmd[prgIdx].motionChangeCmd != motorData[0].b[3]) { // mCmd.cmd

      //Serial.println("TRIGGER END COMMAND CHANGE");
      // if we are avoiding an obstacled with a slowdown curve need to speed up
      //if(leftFrontSideObstacleDetectedStatus || rightFrontSideObstacleDetectedStatus) {
        // send a speed up command to motor slave
      //  sendMotorSlaveMotionCmd(EXT_CMD_SPEED, STD_SIDE_OBSTACLE_SPEED_DIF, 0, 0, 0, 0);
      //}
       
      // allow obstacle detection if not stop
      if(prgCmd[prgIdx].motionChangeCmd != EXT_CMD_STOP) {
        #if !PLOTTER_OUTPUT
          Serial.println("TRIGGER DETECTED RESET frontObstacleDetectedStatus");
        #endif
        frontObstacleDetectedStatus = 0;
        leftFrontSideObstacleDetectedStatus = 0;
        rightFrontSideObstacleDetectedStatus = 0;
      }
      
      #if !PLOTTER_OUTPUT
        Serial.println("RESET frontObstacleDetectedStatus");
      #endif
      leftSideObstacleDetectedStatus = 0;
      rightSideObstacleDetectedStatus = 0;
      //leftFrontSideObstacleDetectedStatus = 0;
      //rightFrontSideObstacleDetectedStatus = 0;
      
      // set the new changeCmd
      if(prgCmd[prgIdx].motionCmd == motorData[0].b[3]) {
        // return to main cmd so set motionChangeCmd to zero;
        prgCmd[prgIdx].motionChangeCmd = 0;
      } else {
        // still in another task update motionChangeCmd to current
        prgCmd[prgIdx].motionChangeCmd = motorData[0].b[3];
      }
    }
  }
}

void detectCmdReception() {
  if(motionCmdRequest && !motorData[0].b[1]) { // reportCmdReceivedStatus
    #if !PLOTTER_OUTPUT
      Serial.println("COMMAND CONFIRMED");
      //displayMotorInfo();
      //Serial.print(motionCmdWaitingId); Serial.println("\t");
    #endif
    motionCmdRequest = false;
  }
}

void detectMotionEnd() {
  if(motionCmdWaitingId && !motorData[0].b[0] && !motionCmdRequest) { // motionCurrentDirection 
    #if !PLOTTER_OUTPUT
      Serial.println("MOTION END DETECTED");
      //displayMotorInfo();
      //Serial.print(motionCmdWaitingId); Serial.println("\t");
    #endif
    cmdEnd();
    motionCmdWaitingId = 0;
    // set flag for odometry
    lastMoveData = true;
  }
}

void motionCmdStart() {
  switch(prgCmd[prgIdx].motionCmd) {
    case EXT_CMD_RUN:
    case EXT_CMD_ROTATE:
    case EXT_CMD_CURVE:
      sendMotorSlaveMotionCmd(prgCmd[prgIdx].motionCmd, prgCmd[prgIdx].v1, prgCmd[prgIdx].v2, prgCmd[prgIdx].v3, prgCmd[prgIdx].v4, prgCmd[prgIdx].v5);
      // set control
      motionCmdStatus = 1;
      break;
  }
}

void cmdStart() {  

  if(prgCmd[prgIdx].cmd > 0) {

    // there is a external command?
    if(prgCmd[prgIdx].motionCmd > 0) {
      motionCmdRequestId++;
      motionCmdStart();
    }

    if(prgCmd[prgIdx].intCmd > 0) { 
      // set control
      intCmdStatus = 1;
    }

    // set control
    cmdStatus = 1;

    #if !PLOTTER_OUTPUT
      Serial.print("START cmd: "); 
      Serial.print(prgCmd[prgIdx].cmd); Serial.print("\t"); 
      Serial.print(prgCmd[prgIdx].intCmd); Serial.print("\t");
      Serial.print(prgCmd[prgIdx].motionCmd); Serial.print("\t");
      Serial.print(prgCmd[prgIdx].v1); Serial.print("\t"); 
      //Serial.print(prgCmd[prgIdx].v2); Serial.print("\t"); 
      //Serial.print(prgCmd[prgIdx].v3); Serial.print("\t"); 
      //Serial.print(prgCmd[prgIdx].v4); Serial.print("\t"); 
      //Serial.print(prgCmd[prgIdx].v5); Serial.print("\t"); 
      Serial.println("");
    #endif
    
  }
}

void cmdEnd() {
  boolean doPrgIdxUpdate = true;
  // TODO: wait for last wheel to stop (to get current total pulses)
  //Serial.println("END");

  // log current command with current distance done
  //float distanceDone = getDistanceDone(i2cData[0][3].l, i2cData[0][4].l);
  //logCmd(prgCmd[prgIdx].extCmd, prgCmd[prgIdx].value1, distanceDone);

  // check what is the current internal command
  
  switch(prgCmd[prgIdx].intCmd) {
    case INT_CMD_GOTO:
      // GOTO prgIdx command
      #if !PLOTTER_OUTPUT
        //Serial.print("GOTO "); Serial.println(prgCmd[prgIdx].value1);
      #endif
      doPrgIdxUpdate = false;
      // set next prgIdx
      prgIdx = prgCmd[prgIdx].v1;
      break;
    case INT_CMD_PAUSE: 
      // PAUSE advance in program index
      delay(250);
      // delay a bit and just keep execute pause
      doPrgIdxUpdate = false;
      #if !PLOTTER_OUTPUT
        //Serial.println("PAUSE");
      #endif
      break;
    case INT_CMD_RESTART:
      // RESTART command sequence (keep old commands)
      doPrgIdxUpdate = false;
      prgIdx = 0;
      #if !PLOTTER_OUTPUT
        //Serial.println("RESTART");
      #endif
      break;    
  }


  // the motor slave terminate the motion
  //if(slaveCmdTerminateId) {
    // odometry
    //lastMoveData = true;
  //}

  if(doPrgIdxUpdate) {
    // advance command sequence
    prgIdx++;
    if(prgIdx == PRG_CMD_MAX || !prgCmd[prgIdx].cmd) {
      // reset command sequence (clean old commands)
      resetPrgCmd();
    }
  }

  // allow new command processing
  cmdStatus = 0;
  //intCmdStatus = 0; // should be all zero here
  motionCmdStatus = 0;
}

void resetPrgCmd() {
  int i;
  Serial.println("resetPrgCmd");
  // clean sequence
  for(i=0; i < PRG_CMD_MAX; i++){
    prgCmd[i] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  }
  // set control vars
  prgIdx = 0;
}
