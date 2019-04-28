#define TIMEOUT_RACE    130000

#define USPINT1 45
#define USPINE1 44
#define USTOUT1 2000 // usec
#define USDMAX1 150

#define USPINT2 A13
#define USPINE2 A12
#define USTOUT2 2000
#define USDMAX2 150

#define USPINT3 A15
#define USPINE3 A14
#define USTOUT3 2000
#define USDMAX3 150

#define USPINT4 43
#define USPINE4 42
#define USTOUT4 2000
#define USDMAX4 150

#define USPINT5 41
#define USPINE5 40
#define USTOUT5 2000
#define USDMAX5 150

#define USPINT6 53
#define USPINE6 52
#define USTOUT6 2000
#define USDMAX6 150

#define USPINT7 51
#define USPINE7 50
#define USTOUT7 2000
#define USDMAX7 150

#define USPINT8 49
#define USPINE8 48
#define USTOUT8 2000
#define USDMAX8 150

#define USPINT9 47
#define USPINE9 46
#define USTOUT9 2000
#define USDMAX9 150

#define TICK_TIME_STATE_MACHINE 10

int usd1, usd2, usd3, usd4, usd5, usd6, usd7, usd8, usd9;
unsigned long prevMillis_tick_mouse_state_machine = 0;

void setup() {
  Serial.begin(115200);  
  SonarSetup(USPINT1, USPINE1);
  SonarSetup(USPINT2, USPINE2);
  SonarSetup(USPINT3, USPINE3);
  SonarSetup(USPINT4, USPINE4);
  SonarSetup(USPINT5, USPINE5);
  SonarSetup(USPINT6, USPINE6);
  SonarSetup(USPINT7, USPINE7);
  SonarSetup(USPINT8, USPINE8);
  SonarSetup(USPINT9, USPINE9);  
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - prevMillis_tick_mouse_state_machine >= TICK_TIME_STATE_MACHINE) {
    prevMillis_tick_mouse_state_machine = currentMillis;
    SonarScan();
  }
}

void SonarSetup(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

int SonarPing(int triggerPin, int echoPin, int timeout) {
  float duration;
  int distance;
  digitalWrite(triggerPin, LOW);      // clean line to trigger
  delayMicroseconds(2);               // Added this line
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);              // trigger duration
  digitalWrite(triggerPin, LOW);      //stop trigger
  duration = pulseIn(echoPin, HIGH, timeout);  //receive duration of pulse
  ///Serial.print(" duration "); Serial.print(duration, 0);
  distance = (duration / 2) / 29 * 10; // compute distance in mm
  if (distance == 0) distance = 11111; // If timeout, reading is 0. Convert to infinite
  return distance;
}

void SonarScan() {
  usd1 = SonarPing(USPINT1, USPINE1, USTOUT1);
  usd2 = SonarPing(USPINT2, USPINE2, USTOUT2);
  usd3 = SonarPing(USPINT3, USPINE3, USTOUT3);
  usd4 = SonarPing(USPINT4, USPINE4, USTOUT4);
  usd5 = SonarPing(USPINT5, USPINE5, USTOUT5);
  usd6 = SonarPing(USPINT6, USPINE6, USTOUT6);
  usd7 = SonarPing(USPINT7, USPINE7, USTOUT7);
  usd8 = SonarPing(USPINT8, USPINE8, USTOUT8);
  usd9 = SonarPing(USPINT9, USPINE9, USTOUT9);
  
  Serial.print(usd1); Serial.print("\t");
  Serial.print(usd2); Serial.print("\t");
  Serial.print(usd3); Serial.print("\t");
  Serial.print(usd4); Serial.print("\t");
  Serial.print(usd5); Serial.print("\t");
  Serial.print(usd6); Serial.print("\t");
  Serial.print(usd7); Serial.print("\t");
  Serial.print(usd8); Serial.print("\t");
  Serial.print(usd9); Serial.print("\t");
  Serial.println("");    
}
