#include <Wire.h>
#define stepper_pin_dir 2
#define stepper_pin 3
#define pulseSpeed 1500
#define stepsPerRevolution 200

#define stepper_time 80

byte stepper_dir = LOW; // anti cockwise
int stepper_t = stepper_time + 1;
int stepper_dt = 0;
bool stepper_working = false;

void setup() {
  pinMode(stepper_pin, OUTPUT);
  pinMode(stepper_pin_dir, OUTPUT);
  Serial.begin(115200);
  Wire.begin(9);
  Wire.onReceive(receiveEvent);
}

void loop() {
  if (stepper_working) loop_stepper();
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

void stop_stepper() {
  Serial.print("Stepper Stopped. Time Taken(s): "); Serial.println(stepper_dt/1000.0);
  digitalWrite(stepper_pin, LOW); digitalWrite(stepper_pin_dir , LOW);
  stepper_dt = millis() - stepper_t;
  stepper_working = false;
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

void receiveEvent(int data) {
  if (Wire.available()) {
    char ch;
    ch = Wire.read();
    if ( ch == '9' ) stop_stepper();
    if ( ch == '1' ) start_stepper(HIGH);
    if ( ch == '0' ) start_stepper(LOW);
  }
}
