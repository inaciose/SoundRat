#define IR_RECEIVER_PIN  12
#define IR_TOWER_SAMPLES 100
int ir_beacon_ang = 0;
boolean ir_beacon_found = false;

#include <Servo.h>
#define IR_SERVO_PIN     6
#define IR_SERVO_TIMER_TICKS 100;
Servo ir_servo;
unsigned long ir_servo_timer;

enum State_journey_enum { STATEJ_FIND_IR_TOWER, STATEJ_FIND_BEGINING, STATEJ_TIMEOUT_RACE};
uint8_t journey_state = STATEJ_FIND_IR_TOWER;

byte ir_found_stats[180]; // test only

void setup() {
  Serial.begin (115200); Serial.println ("Setup ");
  pinMode(IR_RECEIVER_PIN, INPUT);
  // bof test only
  int f;
  for(f = 0; f < 180; f++) {
    ir_found_stats[f] = 0;
  }
  // eof test only
  pinMode(IR_SERVO_PIN, OUTPUT);
  ir_servo.attach(IR_SERVO_PIN);
  ir_servo.write(90);
  delay(500); // testing
  ir_servo_timer = millis() + IR_SERVO_TIMER_TICKS;
}

void loop() {
  
  int f;
  if (journey_state == STATEJ_FIND_IR_TOWER) {
    if(millis() >= ir_servo_timer) {
      sweepIR();
      if(ir_beacon_found) {
        // bof test only
        ir_found_stats[ir_beacon_ang+90]++;
        Serial.print("found: "); Serial.print(ir_beacon_ang); Serial.println("\t");         
        for(f = 10; f < 165; f+=5) {
          Serial.print(ir_found_stats[f]); Serial.print("\t"); 
        }        
        // eof test only
        Serial.println("");
      }
      ir_servo_timer = millis() + IR_SERVO_TIMER_TICKS;
    }
  }
}

void sweepIR() {
  // Update global ir_beacon_ang
  // Update global ir_beacon_found  

  boolean ir_found = false;
  static int servo_old_pos = 89;
  static int servo_current_pos = 90;
  unsigned char ir_read = 0;
  
  for(int i=0; i<IR_TOWER_SAMPLES; i++){
    ir_read += !digitalRead(IR_RECEIVER_PIN);  //Negative logic
    delayMicroseconds(10);
  }
  Serial.print(servo_current_pos); Serial.print("\t"); Serial.print(ir_read); Serial.println(""); // test only
  
  // IR Detect tower
  if (ir_read > IR_TOWER_SAMPLES/4) {
    // Convert ang from servo range (0ª; 180º) to robot range(-90º; 90ª) 
    ir_beacon_ang = servo_current_pos-90;
    ir_found = true;
  }

  // update servo position for next iteraction
  if (servo_old_pos < 154 && servo_old_pos < servo_current_pos) {
    servo_old_pos += 5;
    servo_current_pos += 5;
  } else if (servo_current_pos >= 155) {
    servo_current_pos = 150;
    servo_old_pos = 155;
  } else if (servo_current_pos <= 10) {
    servo_current_pos = 15;
    servo_old_pos = 10;
  } else if (servo_old_pos > 9 && servo_old_pos > servo_current_pos) {
    servo_old_pos -= 5 ;
    servo_current_pos -= 5;
  }
  
  // send new position to servo
  ir_servo.write(servo_current_pos);

  //delay(50);
  ir_beacon_found = ir_found;
  ir_servo_timer = millis() + IR_SERVO_TIMER_TICKS;
}
