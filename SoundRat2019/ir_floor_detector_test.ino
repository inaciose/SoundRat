#define IR_SENSOR_NUM_SENSORS 3
#define IR_SENSOR_NUM_READINGS 10

#define IR_FLOOR_PIN_S0 A0
#define IR_FLOOR_PIN_S1 A1
#define IR_FLOOR_PIN_S2 A2

#define IR_FLOOR_S0_TH  400   //Higher values -> Higher reflection
#define IR_FLOOR_S1_TH  400   //Higher values -> Higher reflection
#define IR_FLOOR_S2_TH  400   //Higher values -> Higher reflection

const unsigned char ir_floor[IR_SENSOR_NUM_SENSORS][2] = {
    {IR_FLOOR_PIN_S0, IR_FLOOR_S0_TH},
    {IR_FLOOR_PIN_S1, IR_FLOOR_S1_TH},
    {IR_FLOOR_PIN_S2, IR_FLOOR_S2_TH}
  };

enum State_enum { STATE_STOP, STATE_FORWARD, STATE_ROTATE_RIGHT, STATE_ROTATE_LEFT, STATE_ABORT, STATE_ROTATE_TETA_RIGHT,
                  STATE_ROTATE_TETA_LEFT, STATE_IR_TOWER_DETECTED, STATE_TOWER_ARRIVE, STATE_TIMEOUT_RACE, STATE_BEGIN_RETURN,
                  STATE_ROTATE_IR_SCAN, STATE_ROTATE_180
                };
uint8_t mouse_state = STATE_ABORT;

enum State_journey_enum { STATEJ_FIND_IR_TOWER, STATEJ_FIND_BEGINING, STATEJ_TIMEOUT_RACE};
uint8_t journey_state = STATEJ_FIND_IR_TOWER;

void setup() {
  Serial.begin (115200); Serial.println ("Setup ");
  pinMode(IR_FLOOR_PIN_S0,INPUT);
  pinMode(IR_FLOOR_PIN_S1,INPUT);
  pinMode(IR_FLOOR_PIN_S2,INPUT);
}

void loop() {
  int i_tmp = 0;
  if(journey_state == STATEJ_FIND_IR_TOWER) {
    i_tmp = IR_floor_scan();    
    if (i_tmp){
      mouse_state = STATE_TOWER_ARRIVE;
      //journey_state = STATEJ_FIND_BEGINING; 
    }
  }
}

// IR_floor_scan ////////////////////////////////////////////////////////////
// Read all sensors x times and count number of LOW readings.
// input: 
// out: 0 : white floor. | n: binary number with sensor cobination taht detected black floor.
unsigned char IR_floor_scan() {
  int i_tmp = 0;
  unsigned char ret=0;
  unsigned char ir_floor_sensor[IR_SENSOR_NUM_SENSORS]={0, 0, 0};

  // Scan Sensors
  for(int i=0; i<IR_SENSOR_NUM_READINGS; i++) {       
    for(int s=0; s<IR_SENSOR_NUM_SENSORS; s++) {
      i_tmp = analogRead(ir_floor[s][0]);
      // Read black ?
      if(i_tmp > ir_floor[s][1])  
        ir_floor_sensor[s] += 1;
    }  
  }

  // Evaluate sensors readings
  for(int s=0; s<IR_SENSOR_NUM_SENSORS; s++) {
    // More than 50% readings? 
    if (ir_floor_sensor[s] > IR_SENSOR_NUM_READINGS/2)  
      ret |= 1<<s;
  }
  
  Serial.print (ir_floor_sensor[0]); Serial.print ("\t"); Serial.print (ir_floor_sensor[1]); Serial.print ("\t"); Serial.print (ir_floor_sensor[2]); Serial.print ("\t"); Serial.print (ret); Serial.println ("\t");
  
  return ret; 
}
