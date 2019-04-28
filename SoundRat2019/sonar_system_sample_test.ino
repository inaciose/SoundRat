#include <Wire.h>

// Comunication 4 byte std union
union u_tag {
  byte b[4];
  signed char sc[4];
  int i[2];
  double d;
  long l;
};

byte i2cSlave[1] = {16};
#define I2C_NODE 10
#define I2C_DATA_LEN 32
union u_tag sonarData[I2C_DATA_LEN];

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_NODE);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
}

void loop() {
  getSonarReads();

  Serial.print(sonarData[0].i[0]);
  Serial.print(sonarData[0].i[1]);
  Serial.print(sonarData[1].i[0]);
  Serial.print(sonarData[1].i[1]);
  Serial.print(sonarData[2].i[0]);
  Serial.print(sonarData[2].i[1]);
  Serial.print(sonarData[3].i[0]);
  Serial.print(sonarData[3].i[1]);
  Serial.print(sonarData[4].i[0]);
  Serial.println("");
}

void getSonarReads() {
  byte *ptr = (byte*)&sonarData[0];
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
  }
}
