void getMotorSlaveUpdate() {
  int tc = 0;
  byte *ptr = (byte*)&motorData[0];
  // if more than one slave or page, 
  // we need to handle the pointer to
  // point to correct address
  //Serial.println("request");
  Wire.requestFrom((int)i2cSlave[0], 32);
  //Serial.println("request done");
  while(Wire.available()) {
    byte b = Wire.read();
    *ptr = b;
    ptr++;
    tc++;
    //Serial.println(tc);
  }
  // Odometry
  leftTicks = motorData[3].l; // encoderLeftTotalPulses
  rightTicks = motorData[4].l; // encoderRightTotalPulses
}

void sendMotorSlaveMotionCmd(byte cmd, signed long units, byte start, byte target, byte end, signed char accel) {
  //return;
  Wire.beginTransmission(i2cSlave[0]);
  Wire.write(I2C_NODE);                         // byte: ourNodeId
  Wire.write((byte*)&cmd, 1);                   // byte: basic config cmd
  Wire.write((byte*)&motionCmdRequestId, 1);    // byte
  Wire.write(0);                                // byte PAD
  //
  Wire.write((byte*)&units, 4);                 // long
  //
  Wire.write((byte*)&start, 1);                 // byte
  Wire.write((byte*)&target, 1);                // byte
  Wire.write((byte*)&end, 1);                   // byte
  Wire.write((byte*)&accel, 1);                 // byte
  Wire.endTransmission();
  motionCmdWaitingId = motionCmdRequestId;
  motionCmdRequest = true;
}

void sendMotorSlaveCurveCmd(byte cmd, signed long units, signed int radius, signed char accel) {
  Wire.beginTransmission(i2cSlave[0]);
  Wire.write(I2C_NODE);                         // byte: ourNodeId
  Wire.write((byte*)&cmd, 1);                   // byte: basic config cmd
  Wire.write((byte*)&motionCmdRequestId, 1);    // byte
  Wire.write(0);                                // byte PAD
  //
  Wire.write((byte*)&units, 4);                 // long
  //
  Wire.write((byte*)&radius, 2);                // int
  Wire.write((byte*)&accel, 1);                 // byte
  Wire.endTransmission();
  motionCmdWaitingId = motionCmdRequestId;
  motionCmdRequest = true;
}

/*
// BOF: serial rpi communication
void getSerialData1() {
  boolean complete = false;
  static byte i = 0;
  static byte rec[SERIAL1_RECBUF_LEN];
  // receive message
  while(Serial1.available() && i < SERIAL1_RECBUF_LEN) {
    //serReceiving[0] = true;
    rec[i] = Serial1.read();
    //Serial.println(rec[i]);
    if(rec[i] == '\n') {
      complete = true;
      rec[i] = 0;
    }
    i++;
  }
  // process full message
  if(complete) {
    byte f;
    //serReceiving[0] = false;  // TODO: remove
    // copy to global var
    for(f = 0; f < SERIAL1_DATA_LEN ; f++) {
      // rpi pyton have inverse byte order
      piData[f].b[3] = rec[f*4 + 0];
      piData[f].b[2] = rec[f*4 + 1];
      piData[f].b[1] = rec[f*4 + 2];
      piData[f].b[0] = rec[f*4 + 3];
    }
    // reset local var
    for(f = 0; f < SERIAL1_RECBUF_LEN; f++) rec[f] = 0;
    i = 0;
    // set control
    //serReceived[0] = true;  // TODO: remove

    for(f = 0; f < 5; f++) { 
      Serial.print("RECEIVED "); 
      Serial.print(piData[0].b[0]); Serial.print("\t");
      Serial.print(piData[0].b[1]); Serial.print("\t");
      Serial.print(piData[0].b[2]); Serial.print("\t");
      Serial.print(piData[0].b[3]); Serial.print("\t");
    }
    Serial.println("");
  }  
}

void sendSerialData1() {
  //Serial1.write(c);
 
  Serial1.print(sonarValue[0] / SONAR_SCALE); Serial1.print(" ");
  Serial1.print(sonarValue[2] / SONAR_SCALE); Serial1.print(" ");
  Serial1.print(sonarValue[5] / SONAR_SCALE); Serial1.print(" ");
  Serial1.print(sonarValue[3] / SONAR_SCALE); Serial1.print(" ");
  Serial1.print(sonarValue[1] / SONAR_SCALE); Serial1.print(" ");
  Serial1.print(sonarValue[4] / SONAR_SCALE); Serial1.print(" ");
  Serial1.print(frontObstacleDetectedStatus); Serial1.print(" ");
  Serial1.print(leftSideObstacleDetectedStatus); Serial1.print(" ");
  Serial1.print(rightSideObstacleDetectedStatus); Serial1.print(" ");
  Serial1.println("");
  //delay(1000);

}
// EOF: serial rpi communication
*/
