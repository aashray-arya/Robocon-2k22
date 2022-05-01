#include <Servo.h>
#include <Wire.h>

#define servoPin 3 // signal pin for the ESC.
Servo servo;

int i=1700;

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
  if(x==0){
    servo.writeMicroseconds(1000);
    i=1700;
    Serial.println(x);
  } else{
    i++;
    if(i>1850)
    i=1850;
    servo.writeMicroseconds(i);
    Serial.println(x);
  }
}
