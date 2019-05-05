#define PLOTTER_OUTPUT false
#define SERIAL_SPEED 115200

#include <Wire.h>
#include "Kalman.h"

// Comunication 4 byte std union
union u_tag {
  byte b[4];
  signed char sc[4];
  int i[2];
  double d;
  long l;
};

/*
#include <Servo.h>
Servo frontServo;
#define FRONT_SERVO_PIN 5
#define FRONT_SERVO_STEP 4
int frontServoAngle = 0;
boolean frontServoGoLeft = false;
*/

//
// SENSOR
//

/*
#include <VL53L0X.h>
VL53L0X frontLidar;
#define FRONT_LIDAR_INTERVAL 32
int frontLidarRead = 8190;
unsigned long frontLidarTimer = 0;
*/

// BOF: SONAR
#include <NewPing.h>
#define SONAR_NUM           6             // Number of sensors.
// six sonar config
#define SONAR_LEFT_LEFT 0
#define SONAR_LEFT_FRONT 2
#define SONAR_FRONT_LEFT 5
#define SONAR_FRONT_RIGHT 3
#define SONAR_RIGHT_FRONT 1
#define SONAR_RIGHT_RIGHT 4
// other config
#define SONAR_SCALE         10            // 1 for cm / 10 for mm 
#define SONAR_ZERO_MIN      2             // number of consecutive zero readings to use zero
#define SONAR_MAX_DISTANCE  200           // Maximum distance (in cm) to ping.
#define SONAR_PING_INTERVAL 33            // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SONAR_MAX_ANOMALY   33
unsigned long sonarPingTimer[SONAR_NUM];  // Holds the times when the next ping should happen for each sensor.
unsigned int sonarRead[SONAR_NUM];        // Where the raw ping distances are stored.
unsigned int sonarKalman[SONAR_NUM];      // Where the kalman processed ping distances are stored.
unsigned int sonarValue[SONAR_NUM];       // Where computed current ping distances are stored.
unsigned int sonarZeroCounter[SONAR_NUM]; // Where the consecutive zero ping distances count are stored.
byte sonarAnomalyCounter[SONAR_NUM]; 
uint8_t sonarIndex = 0;                   // Holds the current sonar sensor number
NewPing sonar[SONAR_NUM] = {              // Sensor object array.
  // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(22, 22, SONAR_MAX_DISTANCE), //SONAR_LEFT_LEFT
  NewPing(24, 24, SONAR_MAX_DISTANCE), //SONAR_RIGHT_FRONT
  NewPing(23, 23, SONAR_MAX_DISTANCE), //SONAR_LEFT_FRONT
  NewPing(26, 28, SONAR_MAX_DISTANCE), //SONAR_FRONT_RIGHT
  NewPing(27, 27, SONAR_MAX_DISTANCE), //SONAR_RIGHT_RIGHT
  NewPing(25, 25, SONAR_MAX_DISTANCE), //SONAR_FRONT_LEFT
};
Kalman sonarKalman0(0.05,0.05,1023,100);
Kalman sonarKalman1(0.05,0.05,1023,100);
Kalman sonarKalman2(0.05,0.05,1023,100);
Kalman sonarKalman3(0.05,0.05,1023,100);
Kalman sonarKalman4(0.05,0.05,1023,100);
Kalman sonarKalman5(0.05,0.05,1023,100);
// EOF: SONAR

#define READ_POSITION_MAX 13
byte readPosition[READ_POSITION_MAX] = {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180};
int readPositionDistance[READ_POSITION_MAX] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//
// ODOMETRY
//
//units: mm for high resolution / cm for low resolution
//#define ENCODER_PULSES 1920.0 // encoder resolution
#define ENCODER_PULSES 1920.0
#define WHEEL_RADIUS 40.5 // wheel diameter / 2
#define WHEEL_AXIS_RADIUS 95.0 // = lenght of axis / 2

int bodyEncoderPulses = ENCODER_PULSES;
float bodyWheelRadius = WHEEL_RADIUS;
float bodyWheelAxisRadius = WHEEL_AXIS_RADIUS;
float bodyEncoderPulsesPerUnit = 2 * PI * bodyWheelRadius / bodyEncoderPulses;

//
// I2C
//

byte i2cSlave[1] = {11};
#define I2C_NODE 10
#define I2C_DATA_LEN 32
union u_tag motorData[I2C_DATA_LEN];

//
// COMMAND CONTROL
//

#define CMD_GENERIC 254

#define REC_CMD_RESET 255
#define REC_CMD_STOP_NOW 1
#define REC_CMD_RUN_NOW 2

#define EXT_CMD_STOP 1
#define EXT_CMD_RUN 2
#define EXT_CMD_ROTATE 3
#define EXT_CMD_CURVE 4
#define EXT_CMD_SPEED 5

#define INT_CMD_SCAN_FRONT 150
#define INT_CMD_DECIDE 250
#define INT_CMD_GOTO 251
#define INT_CMD_PAUSE 252
#define INT_CMD_RESTART 253

// some standart motion parameters
#define STD_RUN_DISTANCE 1000
#define STD_RUN_NOWAY_DISTANCE 500
#define STD_TARGET_SPEED 40
#define STD_SIDE_OBSTACLE_SPEED_DIF 5

#define STD_SIDE_CURVE_ANGLE 45
#define STD_SIDE_CURVE_RADIUS 250

// commands sequence
#define PRG_CMD_MAX 4

typedef struct {
  byte cmd;
  byte intCmd;
  byte motionCmd;
  byte motionChangeCmd;
  long v1;
  byte v2;
  byte v3;
  byte v4;
  signed char v5;
  signed int radius;
} cmd_t;

cmd_t prgCmd[PRG_CMD_MAX] = {
  {CMD_GENERIC, 0, EXT_CMD_RUN, 0, 0, 0, 0, 0, 0, 0},
};

byte prgIdx = 0;

byte cmdStatus = 0;
byte intCmdStatus = 0;
byte motionCmdStatus = 0;
byte motionCmdRequestId = 0;
byte motionCmdWaitingId = 0;
boolean motionCmdRequest = false;

boolean extStopCmdStatus = false;
boolean intStopCmdStatus = false;

// TODO change to: stopedMotionCmd / stopedMotionCmdUnits 
//byte stopedExtCmd = 0;
//signed long stopedExtCmdUnits = 0;

byte idleLoopCounter = 0;
#define IDLE_LOOP_COUNTER_MAX 120


// ODOMETRY
#include <Odometry.h>
#define POSITION_COMPUTE_INTERVAL 50 // ms
#define SEND_INTERVAL 100 // ms

boolean lastMoveData = false;

// Cartesian coordinate of latest location estimate.
// Length unit correspond to used on wheel and whell axis radius.
double px = 0;
double py = 0;

// Left and right angular velocities.
double wl = 0;
double wr = 0;

// getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
// This angle is set initially at zero before the robot starts moving.
double theta = 0;

// Total distance robot has troubled.
double pd = 0;

// Number of left and right tick counts on the encoder.
signed int leftTicks, rightTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;

Odometry Odometry(&leftTicks, &rightTicks, ENCODER_PULSES, WHEEL_RADIUS, WHEEL_AXIS_RADIUS * 2);


//
// PI COMMS
//
/*
#include <SoftwareSerial.h>
SoftwareSerial Serial1(10, 11); // RX, TX
#define SERIAL1_RECBUF_LEN 16
#define SERIAL1_DATA_LEN 8
union u_tag piData[SERIAL1_DATA_LEN];
cmd_t piCmd = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
*/

//
// GENERAL
//

// visual feedback
#define LED_PIN 13 
boolean internalLedBlinkState = false;
int internalLedBlinkDelay = 1000;

// loop control
unsigned long loopTimeStart;
byte loopTime;

// reset function
void (* resetFunc) (void) = 0;

// obstacle avoidance
byte frontObstacleDetectedStatus = 0;
byte leftSideObstacleDetectedStatus = 0;
byte rightSideObstacleDetectedStatus = 0;
byte leftFrontSideObstacleDetectedStatus = 0;
byte rightFrontSideObstacleDetectedStatus = 0;

#define WALL_SIDE_THRESHOLD 100
#define WALL_SIDE_FRONT_THRESHOLD 80
#define WALL_SIDE_ROTATE_THRESHOLD 170
#define WALL_FRONT_THRESHOLD 200
#define WALL_FRONT_ALLEY_THRESHOLD 400


void setup() {
  Serial.begin(SERIAL_SPEED);

  #if !PLOTTER_OUTPUT
  //Serial.println("STARTING...");
  #endif

  //Serial1.begin(2400);

  #if !PLOTTER_OUTPUT
  //Serial.println("SOFTSERIAL done");
  #endif

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(I2C_NODE);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  #if !PLOTTER_OUTPUT
  //Serial.println("I2C done");
  #endif

  // do some previous sonar readings
  setupSonar();

  #if !PLOTTER_OUTPUT
  //Serial.println("SONAR done");
  #endif
/*
  delay(50);
  motionCmdRequestId = 1;
  //slave using i2c master writer
  Wire.beginTransmission(i2cSlave[0]);  
  Wire.write(I2C_NODE); // ourNodeId
  Wire.write(2);
  Wire.write((byte*)&motionCmdRequestId, 1);
  Wire.write(0);
  Wire.endTransmission();
  // does not update slaveCmdRequestId
  // allow some time to controller reply
  delay(2);
  
  getMotorSlaveUpdate();

  displayMotorInfo()  
  Serial.println("");
*/
  
  #if !PLOTTER_OUTPUT
  //Serial.println("MOTOR done");
  #endif
  
  #if !PLOTTER_OUTPUT
  //Serial.println("ALL done");
  #endif

  // servo  
  /*
  frontServo.attach(FRONT_SERVO_PIN);
  frontServo.write(176);
  delay(100);

  // lidar
  frontLidar.init();
  frontLidar.setTimeout(500);
  frontLidar.startContinuous();
  frontLidarTimer = millis() + FRONT_LIDAR_INTERVAL;
  */
  
  loopTimeStart = millis();

}

void loop() {
  static boolean internalLedBlinkState = false;
  static unsigned long internalLedTimeStart = millis() + internalLedBlinkDelay;
  int i;

  cmdProcess(); 
  //Serial.println("cmdProcess() done");
  

  if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
    // Computes the new angular velocities and uses that to compute the new position.
    // The accuracy of the position estimate will increase with smaller time interval until a certain point.
    Odometry.computePosition();
    prevPositionComputeTime = millis();
    //Serial.println("odo1 done");
  }  

  if (millis() - prevSendTime > SEND_INTERVAL) {
    // Cartesian coordinate of latest location estimate.
    // Length unit correspond to used on wheel and whell axis radius.
    px = Odometry.getX();
    py = Odometry.getY();

    // Left and right angular velocities.
    wl = Odometry.getWl();
    wr = Odometry.getWr();

    // getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
    // This angle is set initially at zero before the robot starts moving.
    theta = Odometry.getTheta();

    // Total distance robot has troubled.
    pd = sqrt(px * px + py * py);

    /*
    Serial.print(i2cData[0][0].b[0]); Serial.print(" "); // requestEventInfoType
    Serial.print(i2cData[0][0].b[1]); Serial.print(" "); // bodyMoveProcessStatus
    Serial.print(i2cData[0][0].i[1]); Serial.print(" "); // bodyMoveCmd.id
    Serial.print(i2cData[0][1].b[0]); Serial.print(" "); // bodyMoveCmd.cmd
    Serial.print(i2cData[0][1].b[1]); Serial.print(" "); // loopTime
    Serial.print(i2cData[0][3].l); Serial.print("\t"); // bodyEncoderLeftTotalPulses
    Serial.print(i2cData[0][4].l); Serial.print("\t"); // bodyEncoderRightTotalPulses
    Serial.print(leftTicks * bodyEncoderPulsesPerUnit); Serial.print("\t");
    Serial.print(pd); Serial.print("\t");
    Serial.print(py); Serial.print("\t");
    Serial.print(wl); Serial.print("\t");
    Serial.print(wr); Serial.print("\t");
    Serial.print(theta*RAD_TO_DEG); Serial.print("\t");
    Serial.print(pd); Serial.print("\t");
    Serial.println("");
    */
    
    // set control
    if(lastMoveData) {
      leftTicks = 0;
      rightTicks = 0;      
      Odometry.resetLastTickets();
      //motorData[0].b[0] = 0; // requestEventInfoType
      //motorData[0].b[1] = 0; // requestEventInfoType
      // should I reset these?
      motorData[3].l = 0; // bodyEncoderLeftTotalPulses
      motorData[4].l = 0; // bodyEncoderRightTotalPulses
      lastMoveData = false;
    }
  
    prevSendTime = millis();
    //Serial.println("odo2 done");

  }
/*
  if(millis() > frontLidarTimer) {
    frontLidarTimer = millis() + FRONT_LIDAR_INTERVAL;

    frontLidarRead = frontLidar.readRangeContinuousMillimeters();
    if (frontLidar.timeoutOccurred()) { Serial.print(" TIMEOUT: lidarRead"); }
    if(frontLidarRead > 2000) frontLidarRead = 0;
  
    frontServo.write(frontServoAngle);
    if(frontServoGoLeft) {
      frontServoAngle += FRONT_SERVO_STEP;
      if(frontServoAngle >= 176) frontServoGoLeft = false;
    } else {
      frontServoAngle -= FRONT_SERVO_STEP;
      if(frontServoAngle <= 4) frontServoGoLeft = true;
    }
  }
*/
  
  processSonar();
  //Serial.println("processSonar() done");
  

  obstacleDetection();
  //Serial.println("obstacleDetection() done");

/*
  // pool pi serial line to
  // send and receive data
  sendSerialData1();
  getSerialData1();

  if(piData[0].l) {    
    // process imediate commands
    switch(piData[0].l) {
      case REC_CMD_RESET: // reset
        //Serial.print("RESET CMD"); Serial.println("\t");
        resetFunc();
        break;
      default:
        piCmd.cmd = (byte)piData[0].l;
        break;
    }
    // reset remote received data from Pi
    for(int f = 0; f < SERIAL1_DATA_LEN; f ++) piData[f].l = 0;   
  }
*/
  // blink LED to indicate activity
  if(millis() >= internalLedTimeStart) {
    internalLedTimeStart = millis() + internalLedBlinkDelay;
    internalLedBlinkState = !internalLedBlinkState;
    digitalWrite(LED_PIN, internalLedBlinkState);
  }

  // loopTime control
  loopTime = millis() - loopTimeStart;
  loopTimeStart = millis();

  displayLoopData();
  //Serial.println("loop done");
}
