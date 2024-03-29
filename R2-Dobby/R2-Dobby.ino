/*
  External Library Requiremens:
    DualShock4_lib     ->      https://github.com/nikh1508/DualShock4_lib
    PID_v1             ->      https://github.com/br3ttb/Arduino-PID-Library
  Rest of the libraries are included in /src folder
*/
#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <DualShock4_lib.h>
#include "src/Encoder/Encoder.h"
#include "src/IMU/IMU.h"
#include "src/HolonomicDrive/HolonomicDrive.h"
#include <Servo.h>
#include <String.h>

//# define X_EN_5v  53 //ENA+(+5V) stepper motor enable , active low     Orange
//# define dirPin 49 //DIR+(+5v) axis stepper motor direction control  Brown
//# define stepPin 45//PUL+(+5v) axis stepper motor step control       RED

#define relay11 27
#define relay12 29
#define relay21 25
#define relay22 23

#define relay31 A4 //lagori gripper motor p1
#define relay32 A6 //lagori gripper motor p2

//#define in1DC 49
//#define in2DC 53
#define pwmDC 7 //lagori gripper motor pwm

#define lpwm 10  //ball picking mechanism bts7960 motor
#define rpwm 9

#define PW1 37
#define PW2 35
int flagg = 0;
int flaggg = 0;

#define dc1 A1
#define dc2 A3
#define dcpwm 8
float kp, ki, kd;
bool power;


Servo myservo1;
Servo myservo2;

constexpr int MAX_SPEED = 40;
constexpr int AXIS_DEAD_ZONE = 1500;
constexpr int AXIS_DEAD_ZONE_L = 1500;
int pos, i;
//  Function Prototypes:
void setup();
void loop();
bool checkForRoutines();
void forestRoutine();
void decodeData(byte toDecode[], byte totalRecvd, byte &decodedByteCount, byte decodedBytes[], byte specialByte);
bool ReadBytes(byte data[], byte toRead, char toSend);
void convert(byte toConvert[], unsigned int converted[], long &res);
bool funcData(double values[]);
void Serial3Flush();

constexpr motor front(A9, A7, 6);  //DO | D1 | PWM        //CHANGE-HERE
constexpr motor left(A13, A15, 4); //CHANGE-HERE
constexpr motor right(A12, A10, 5);   //CHANGE-HERE

//  PID Constants
constexpr double linearConst[] = {0.0, 0.0, 0.0};          //Kp | Ki | Kd
constexpr double rotationalConst[] = {0.04215, 0.02, 0.01}; //{0.03, 0.135, 0.011} //avg:{0.03, 0.117, 0.018}

HolonomicDrive bot(front, left, right, linearConst, rotationalConst, MAX_SPEED); //Front | Left | Right | Linear Constant | Rotational Constant | Max Speed
DualShock4 ds4(Serial3);

bool powerOn = false;

constexpr int powerLed = 53;
//PIDTuner tuner(&powerOn, &kp, &ki, &kd, Serial3);a
int Lx, Ly, Rx, Ry;

uint8_t routineCounter = 0;
bool checkForRoutines()
{
  if (ds4.buttonPressed(UP) && routineCounter == 0)
  {
    forestRoutine();
    routineCounter++;
    return true;
  }
  return false;
}

void forestRoutine() {};

void testRoutine ()
{
  static Coordinate pos, next;
  const int VEL = 50;
  const float maxX = 35000.0f;
  const float change = 14800.0f;
  float _tan = 0.0f;
  int _vel = 0;
  float A = 7300.0;
  float B = 0.00038; //0.000275
  float shift = 0.85f;

  getCoordinates(pos);


  if (next.x < maxX)
  {
    _tan = toDegree(atan(A * B * cos(B * next.x))) + 90;
    if (pos.x >= next.x)
    {
      _vel = VEL;
      next.x += 100;
      next.y = A * sin(B * (next.x + shift));
    }
    else
    {
      _vel = VEL;
    }
  }
  else if (next.x < maxX)
  {
    A = 7600.0;
    B = 0.00038;
    shift = 0.85f;
    _tan = toDegree(atan(A * B * cos(B * next.x))) + 90;
    if (pos.x >= next.x)
    {
      _vel = VEL;
      next.x += 100;
      next.y = A * sin(B * (next.x + shift));
    }
    else
    {
      _vel = VEL;
    }
  }
  else
  {
    _vel = 0;
  }

  Serial.println("N.X: " + String(next.x) + "N.Y: " + String(next.y) + "\tVEL: " + String(_vel) + "\tTAN: " + String(_tan) + "\tX: " + String(pos.x) + "\tY: " + String(pos.y));
  bot.move(_vel, _tan);
}
//void servo()
//{
//  myservo.writeMicroseconds(1850);
//  Serial.println("BLDC start");
//}
//void servo1()
//{
//  //   for (int i=pos; i>= 0; i -= 1) { // goes from 0 degrees to 180 degrees
//  //    // in steps of 1 degree
//  myservo.write(0);
//  //    delay(15);}
//}

void Serial3Flush()
{
  while (Serial3.available() > 0)
    char ch = Serial3.read();
}

void decodeData(byte toDecode[], byte totalRecvd, byte &decodedByteCount, byte decodedBytes[], byte specialByte)
{
  for (int i = 1; i < (totalRecvd - 1); i++)
  {
    byte x = toDecode[i];
    if (x == specialByte)
      x += toDecode[++i];
    decodedBytes[decodedByteCount++] = x;
  }
}

bool ReadBytes(byte data[], byte toRead, char toSend)
{
  static bool firstCall = true;
  static bool inProgress = false;
  static byte bytesRecvd = 0;
  static byte tempBuffer[34];
  static long long lastRead;
  byte specialByte = 253;
  byte startMarker = 254;
  byte endMarker = 255;

  if (firstCall)
  {
    firstCall = false;
    Serial3Flush();
    Serial3.print(toSend);
    lastRead = millis();
  }

  while (Serial3.available())
  {
    byte x = Serial3.read();
    //Serial.println(x);
    lastRead = millis();
    if (x == startMarker)
    {
      //Serial.println("SM Recvd");
      inProgress = true;
      bytesRecvd = 0;
    }
    if (inProgress)
      tempBuffer[bytesRecvd++] = x;
    if (x == endMarker)
    {
      //Serial.println("EM Recvd");
      inProgress = false;
      firstCall = true;
      byte decodedByteCount = 0;
      decodeData(tempBuffer, bytesRecvd, decodedByteCount, data, specialByte);
      if (decodedByteCount == toRead)
        return true;
      else
        Serial.println("Wrong Bytes Recvd" + String(decodedByteCount));
    }
  }
  if ((millis() - lastRead) > 50)
  {
    firstCall = true;
    Serial.println("TOO LATE");
  }
  return false;
}

void convert(byte toConvert[], unsigned int converted[], long &res)
{
  converted[0] = int(toConvert[0]);
  for (int i = 0; i < 6; i++)
  {
    converted[i + 1] = (int)toConvert[2 * i + 1] << 8 | ((int)toConvert[2 * i + 2]);
  }
  res = (long)toConvert[13] << 16 | (long)toConvert[14] << 8 | (long)toConvert[15];
}

bool funcData(double values[])
{
  double power, LKp, LKi, LKd, AKp, AKi, AKd;
  unsigned int data[7] = {0};
  long resolution = 0;
  byte recv[16];

  if (ReadBytes(recv, 16, 'x'))
  {
    convert(recv, data, resolution);
    power = values[0] = (double)data[0];
    LKp = values[1] = (double)data[1] / (double)resolution;
    LKi = values[2] = (double)data[2] / (double)resolution;
    LKd = values[3] = (double)data[3] / (double)resolution;
    AKp = values[4] = (double)data[4] / (double)resolution;
    AKi = values[5] = (double)data[5] / (double)resolution;
    AKd = values[6] = (double)data[6] / (double)resolution;

    Serial.println("LKp: " + (String)LKp + "\tLKi: " + (String)LKi + "\tLKd: " + (String)LKd + "\tAKp: " + (String)AKp + "\tAKi: " + (String)AKi + "\tAKd: " + (String)AKd);
    return true;
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);
  Wire.begin();
//  pinMode(stepPin, OUTPUT);
//  pinMode(dirPin, OUTPUT);
  pinMode(PW1, OUTPUT);
  pinMode(PW2, OUTPUT);
  pinMode(dc1, OUTPUT);
  pinMode(dc2, OUTPUT);
  pinMode(dcpwm, OUTPUT);

  pinMode(lpwm, OUTPUT);
  pinMode(rpwm, OUTPUT);

  pinMode(A1, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A1, HIGH);
  digitalWrite(A3, HIGH);
  
//  pinMode(X_EN_5v, OUTPUT);
//  digitalWrite(X_EN_5v, LOW);
  pinMode(relay11, OUTPUT);
  pinMode(relay12, OUTPUT);
  pinMode(relay21, OUTPUT);
  pinMode(relay22, OUTPUT); 
  pinMode(relay31, OUTPUT);
  pinMode(relay32, OUTPUT);
//  pinMode(in1DC,OUTPUT);
//  pinMode(in2DC,OUTPUT);
  
  bot.initialize();
  initializeBNO();
  // resetEncoder();
  myservo1.attach(11);
  myservo2.attach(12);

  pinMode(powerLed, OUTPUT);
  //while (!tuner.update());
  debug_msg("Connecting to Controller");
  while (!ds4.readGamepad());
  debug_msg("Controller Connected");
}

bool mright = false;
bool lastPower = false;
uint8_t mode = -1;
int pwm;
void loop()
{

  if (ds4.readGamepad())
  {
    if (ds4.buttonPressed(PS))
    {
      powerOn = !powerOn;
      digitalWrite(powerLed, powerOn);
      debug_msg("POWER : " + String((powerOn) ? "ON" : "OFF"));

    }


    if (powerOn)
    {
      Lx = map(ds4.axis(LX), 0, 65535, -32768, 32767);
      Ly = map(ds4.axis(LY), 0, 65535, -32768, 32767);
      Rx = map(ds4.axis(RX), 0, 65535, 32767, -32768);
      Ry = map(ds4.axis(RY), 0, 65535, 32767, -32768);
      delay(25);
      Serial.print("Lx :");
      Serial.print(Lx);
      Serial.print("\tLy :");
      Serial.println(Ly);

      if (checkForRoutines())
      {
        return;
      }

      else if (!(Rx > -AXIS_DEAD_ZONE && Rx < AXIS_DEAD_ZONE) || !(Ry > -AXIS_DEAD_ZONE && Ry < AXIS_DEAD_ZONE))
      {
        double theta = toDegree(atan((double)Ry / (double)Rx));
        Rx = map(Rx, -32768, 32767, -128, 127);
        Ry = map(Ry, -32768, 32767, -128, 127);
        int a=getYaw();
        Serial.println(a);
        Serial.print("Rx :");
        Serial.println(Rx);
        unsigned int R = constrain(map2((double)((Rx * Rx) + (Ry * Ry)), 0.0, 16384.0, 0.0, 100.0), 0, 100);
        if (Ry >= 0)
        {
          if (theta <= 0)
            theta += 180.0;
        }
        else
        {
          if (theta >= 0)
            theta += 180.0;
          else
            theta += 360.0;
        }

        // debug_msg("Rx : " + String(Rx) + "\tRy : " + String(Ry) + "\tR : " + String(R) + "\tTheta : " + String(theta));
        //          if (R <= 25)
        //          {
        //            rotational.SetTunings(0.021, 0.015, 0.0032);
        //            if (mode != 0)
        //            {
        //              rotational.resetIntegral();
        //              mode = 0;
        //            }
        //          }
        //          else //if (R <= 60)
        //          {
        //            //0.01325, 0.002, 0.01
        //            rotational.SetTunings(0.021, 0.015, 0.0032);
        //            if (mode != 1)
        //            {
        //              rotational.resetIntegral();
        //              mode = 1;
        //            }
        //          }
        bot.move(R, theta);
        //testRoutine();
      }
      else if (ds4.button(SQUARE))
      {
        Serial.println("SQ");
        digitalWrite(PW1, LOW);
        digitalWrite(PW2, HIGH);

      }
      else if (ds4.button(CIRCLE))
      {
        Serial.println("CIR");
        digitalWrite(PW1, HIGH);
        digitalWrite(PW2, LOW);
      }
      else if (!(Lx > -AXIS_DEAD_ZONE && Lx < AXIS_DEAD_ZONE) || !(Ly > -AXIS_DEAD_ZONE && Ly < AXIS_DEAD_ZONE))
      {
        Serial.println("Left Axis Triggered\t");
        //Serial.println(String(Lx));
        pwm = map(Lx, -32768, 32767, -25, 25);
        bot.Rotate_AK(pwm);
//        resetBNO();
      }
      else if (ds4.button(OPTIONS))
      {
        //        Serial.println("move bot");
        //        bot.move(30, 90);
        digitalWrite(relay11, HIGH);
        digitalWrite(relay12, LOW);
        digitalWrite(relay21, HIGH);
        digitalWrite(relay22, LOW);
          

      }
      else if (ds4.button(SHARE))
      {
        //        Serial.println("bot move back");
        //        bot.move(30, 270);
        digitalWrite(relay11, LOW);
        digitalWrite(relay12, HIGH);
        digitalWrite(relay21, LOW);
        digitalWrite(relay22, HIGH);
      }
      else if (ds4.button(DOWN)) {
        resetBNO();
      }
      else if (ds4.button(HAT_LEFT)) {
        //        Serial.println("1 step forward");
        //        digitalWrite(dirPin, HIGH);
        //        digitalWrite(stepPin, HIGH);
        //        delayMicroseconds(22);
        //        digitalWrite(stepPin, LOW);
        //        delayMicroseconds(22);
        analogWrite(pwmDC,255);
        digitalWrite(relay31, HIGH);
        digitalWrite(relay32, LOW);
                  
//                  digitalWrite(in1DC,LOW);
//                  digitalWrite(in2DC,HIGH);
      }
      else if (ds4.button(HAT_RIGHT)) {
        //        Serial.println("1 step backward");
        //        digitalWrite(dirPin, LOW);
        //        digitalWrite(stepPin, HIGH);
        //        delayMicroseconds(22);
        //        digitalWrite(stepPin, LOW);
        //        delayMicroseconds(22);
         analogWrite(pwmDC,255);
        digitalWrite(relay31, LOW);
        digitalWrite(relay32, HIGH);

          
//           digitalWrite(in1DC,HIGH);
//           digitalWrite(in2DC,LOW);  
      }
      else if (ds4.button(R1))
      {
        Serial.println("r1");
//        digitalWrite(dc1, LOW);
//        digitalWrite(dc2, HIGH);
//        analogWrite(dcpwm, 100);
        analogWrite(lpwm, 0);
        analogWrite(rpwm, 127);
      }
      else if (ds4.button(L1))
      {
        //bot.Rotate_AK(-30);
//        digitalWrite(dc1, HIGH);
//        digitalWrite(dc2, LOW);
//        analogWrite(dcpwm, 100);
        analogWrite(lpwm, 127);
        analogWrite(rpwm, 0);
      }
      else if (ds4.button(R2))
      {
        myservo1.write(180);
        myservo2.write(0);
      }
      else if (ds4.button(L2))
      {
        myservo1.write(90);
        myservo2.write(90);
      }
      else if (ds4.button(TRIANGLE))
      {
        myservo1.write(90);
        myservo2.write(90);
      }
      else {
        //Serial.println("Stopping");
        bot.stopAll();
        digitalWrite(PW1, LOW);
        digitalWrite(PW2, LOW);
        analogWrite(lpwm, 0);
        analogWrite(rpwm, 0);
        digitalWrite(relay31, HIGH);
        digitalWrite(relay32, HIGH);
      }

    }
    else
      bot.stopAll();
  }
}

//  if (tuner.update())
//  {
//    if (lastPower == 0 && powerOn == 1) {
//      //resetBNO();
//    mright = !mright;}
//    rotational.SetTunings(kp, ki, kd);   // Serial.println(String(kp) + "\t" + String(ki) + "\t" + String(kd) + "\t" + String(powerOn));
//    //Serial.println(String(powerOn));
//    if (powerOn)
//    {
//
//      (mright) ? bot.move(40, 0) : bot.move(40, 180);
//      //Serial.println("moving");
//     // bot.move(30, 90);
////      powerOn = !powerOn;
//
////  Serial.println(String(kp) + "\t" + String(ki) + "\t" + String(kd) + "\t" + String(powerOn));
//    }
//    else
//    {// Serial.println("Stopping");
//      bot.stopAll();
//    }
//    lastPower = powerOn;
//
//  }

//Serial.println(lastPower);
