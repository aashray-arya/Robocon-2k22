#include <Servo.h>
#include <Wire.h>

#define servoPin 3 // signal pin for the ESC.
Servo servo;

int x;
boolean flag = true;

void receiveEvent(int HowMany);

void setup()
{
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);       // start serial for output
  servo.attach(servoPin);

}

void loop()
{
  int j;
  if (x == 0) {
    servo.writeMicroseconds(1500);
    Serial.println(x);
    flag=true;
  }
  else {
    if (flag) {
      for (j = 1550; j <= 1850; j++) {
        servo.writeMicroseconds(j);
        Serial.println(j);
        delay(15);
      }
    }
    flag = false;
    servo.writeMicroseconds(j);
  }
}

void receiveEvent(int howMany)
{
  while (1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  x = Wire.read();    // receive byte as an integer
  //  if(x==0){
  //    servo.writeMicroseconds(1500);
  //    Serial.println(x);
  //  } else{
  //    for(int j=1550;j<=1850;j++){
  //      servo.writeMicroseconds(j);
  //      Serial.println(j);
  //      delay(20);
  //    }
  //  }
}
