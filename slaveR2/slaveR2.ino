#include <Servo.h>
#include <Wire.h>

#define servoPin 3 // signal pin for the ESC.
Servo servo;


void setup()
{
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);       // start serial for output
  servo.attach(servoPin);

}

void loop()
{
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  while(1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  if(x==1){
        Serial.println("1 step forward");
        digitalWrite(dirPin, HIGH);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
  } else if (x==0){
    digitalWrite(stepPin, LOW);
    Serial.println("Left stop");
  } else if(x==3){
        Serial.println("1 step backward");
        digitalWrite(dirPin, LOW);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
  } else{
      digitalWrite(stepPin, LOW);
      Serial.println("Right stop");
  }
}
