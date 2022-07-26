#include <Wire.h>
#define stepper_pin_dir 2
#define stepper_pin 3
#define pulseSpeed 1500
#define stepsPerRevolution 200

#define stepper_time 1400
#define stepper_time2 500
#define stepper_time3 300
#define stepper_time4 80

byte stepper_dir = LOW; // anti cockwise
int stepper_t = stepper_time + 1;
int stepper_dt = 0;
bool stepper_working = false;
bool stepper_working2 = false;
bool stepper_working3 = false;
bool stepper_working4 = false;

void setup() {
  pinMode(stepper_pin, OUTPUT);
  pinMode(stepper_pin_dir, OUTPUT);
  Serial.begin(115200);
  Wire.begin(9);
  Wire.onReceive(receiveEvent);
}

void loop() {
  if (stepper_working) loop_stepper();
  if (stepper_working2) loop_stepper2();
  if (stepper_working3) loop_stepper3();
  if (stepper_working4) loop_stepper4();
  
}

void loop_stepper() {
  Serial.println("Stepper Moving.");
  digitalWrite(stepper_pin_dir, stepper_dir);
  for (int x = 0; x < stepsPerRevolution; x++) {
    stepper_dt = millis() - stepper_t;
    if ( stepper_dt >= stepper_time ) {
      stop_stepper();
      break;
    }
    digitalWrite(stepper_pin, HIGH); delayMicroseconds(pulseSpeed);
    digitalWrite(stepper_pin, LOW); delayMicroseconds(pulseSpeed);
  }
}

void loop_stepper2() {
  Serial.println("Stepper Moving2.");
  digitalWrite(stepper_pin_dir, stepper_dir);
  for (int x = 0; x < stepsPerRevolution; x++) {
    stepper_dt = millis() - stepper_t;
    if ( stepper_dt >= stepper_time2 ) {
      stop_stepper();
      break;
    }
    digitalWrite(stepper_pin, HIGH); delayMicroseconds(pulseSpeed);
    digitalWrite(stepper_pin, LOW); delayMicroseconds(pulseSpeed);
  }
}

void loop_stepper3() {
  Serial.println("Stepper Moving3.");
  digitalWrite(stepper_pin_dir, stepper_dir);
  for (int x = 0; x < stepsPerRevolution; x++) {
    stepper_dt = millis() - stepper_t;
    if ( stepper_dt >= stepper_time3 ) {
      stop_stepper();
      break;
    }
    digitalWrite(stepper_pin, HIGH); delayMicroseconds(pulseSpeed);
    digitalWrite(stepper_pin, LOW); delayMicroseconds(pulseSpeed);
  }
}

void loop_stepper4() {
  Serial.println("Stepper Moving4.");
  digitalWrite(stepper_pin_dir, stepper_dir);
  for (int x = 0; x < stepsPerRevolution; x++) {
    stepper_dt = millis() - stepper_t;
    if ( stepper_dt >= stepper_time4 ) {
      stop_stepper();
      break;
    }
    digitalWrite(stepper_pin, HIGH); delayMicroseconds(pulseSpeed);
    digitalWrite(stepper_pin, LOW); delayMicroseconds(pulseSpeed);
  }
}

void stop_stepper() {
  Serial.print("Stepper Stopped. Time Taken(s): "); Serial.println(stepper_dt/1000.0);
  digitalWrite(stepper_pin, LOW); digitalWrite(stepper_pin_dir , LOW);
  stepper_dt = millis() - stepper_t;
  stepper_working = false;
  stepper_working2 = false;
}

void start_stepper(byte dir) {
  if(stepper_working){
    Serial.println("Stepper Already working."); 
    return;
  }
  Serial.println("Stepper Started.");
  stepper_dir = dir;
  stepper_t = millis();
  stepper_working = true;
}

void start_stepper2(byte dir) {
  if(stepper_working2){
    Serial.println("Stepper2 Already working."); 
    return;
  }
  Serial.println("Stepper2 Started.");
  stepper_dir = dir;
  stepper_t = millis();
  stepper_working2 = true;
}

void start_stepper3(byte dir) {
  if(stepper_working3){
    Serial.println("Stepper3 Already working."); 
    return;
  }
  Serial.println("Stepper3 Started.");
  stepper_dir = dir;
  stepper_t = millis();
  stepper_working3 = true;
}

void start_stepper4(byte dir) {
  if(stepper_working4){
    Serial.println("Stepper4 Already working."); 
    return;
  }
  Serial.println("Stepper4 Started.");
  stepper_dir = dir;
  stepper_t = millis();
  stepper_working4 = true;
}
void receiveEvent(int data) {
  if (Wire.available()) {
    char ch;
    ch = Wire.read();
    if ( ch == '9' ) stop_stepper();
    if ( ch == '1' ) start_stepper(HIGH);
    if ( ch == '6' ) start_stepper2(HIGH);
    if ( ch == '8' ) start_stepper3(LOW);
    if ( ch == '0' ) start_stepper(LOW);
    if ( ch == '7' ) start_stepper4(HIGH);
    if ( ch == '5' ) start_stepper4(LOW);
  }
}
