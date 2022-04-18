#ifndef Encoder_H
#define Encoder_H
#include <Arduino.h>
#include <Wire.h>
#include "../Misc/Misc.h"

constexpr int ENC[] = {0x07, 0x08};
constexpr int SAMPLE_RATE = 10;
constexpr int THETA_SAMPLE_RATE = 50;
bool newThetaXY = false;
float calculatedThetaXY = 0.0;

long encoderCount[2] = {0}, lastEncoderCount[2] = {0};

bool readEncoder();

struct Coordinate
{
    float x;
    float y;
    Coordinate()
    {
        x = y = 0.0;
    }
};

void getCoordinates(Coordinate &var, float referenceAngle = 0.0)
{
    readEncoder();
    // var.x = encoderCount[0] * cos(toRadian(45.0 - referenceAngle)) - encoderCount[1] * cos(toRadian(45.0 + referenceAngle));
    // var.y = encoderCount[0] * sin(toRadian(45.0 - referenceAngle)) + encoderCount[1] * sin(toRadian(45.0 + referenceAngle));
    var.x = encoderCount[0];
    var.y = encoderCount[1];
}

Coordinate addCoordinates(Coordinate a, Coordinate b)
{
    Coordinate c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    return c;
}

float getTangent(Coordinate P1, Coordinate P2)
{
    return toDegree(atan((P2.y - P1.y)/(P2.x - P1.x)));
}

void calcTheta(float referenceAngle = 0.0)
{
    static Coordinate lastCoordinate;
    Coordinate now;
    getCoordinates(now, referenceAngle);
    float dx = now.x - lastCoordinate.x;
    float dy = now.y - lastCoordinate.y;
    calculatedThetaXY = toDegree(atan(dy / dx));
    if (dy >= 0)
    {
        if (calculatedThetaXY <= 0)
            calculatedThetaXY += 180.0;
    }
    else
    {
        if (calculatedThetaXY >= 0)
            calculatedThetaXY += 180.0;
        else
            calculatedThetaXY += 360.0;
    }
    //if (!isnan(calculatedThetaXY))
       // debug_msg("NowX : " + String(now.x) + "\tNowY : " + String(now.y) + "\tLastX : " + String(lastCoordinate.x) + "\tLastY : " + String(lastCoordinate.y) + "\tdx : " + String(dx) + "\tdy : " + String(dy) + "\tdy/dx : " + String(dy / dx) + "\tR : " + String(sqrt(dx * dx + dy * dy)) + "\tTH : " + String(calculatedThetaXY));
    lastCoordinate = now;
}

bool readEncoder()
{
    static long lastEncoderRead = millis() - SAMPLE_RATE;
    static long lastThetaCalc = millis() - SAMPLE_RATE;
    long currentTime = millis();
    if ((currentTime - lastEncoderRead) < SAMPLE_RATE)
        return false;
    lastEncoderRead = currentTime;
    byte recv;
    for (int i = 0; i < 2; i++)
    {
        Wire.requestFrom(ENC[i], 4);
        encoderCount[i] = 0;
        for (int j = 0; j < 4; j++)
        {
            recv = Wire.read();
            encoderCount[i] = encoderCount[i] << 8 | recv;
        }
        encoderCount[i] *= -1;
    }
    // debug_msg("Enc_0 : " + String(encoderCount[0]) + "\tEnc_1 : " + String(encoderCount[1]));
    currentTime = millis();
    if ((currentTime - lastThetaCalc) > THETA_SAMPLE_RATE)
    {
        newThetaXY = true;
        calcTheta();
        lastThetaCalc = currentTime;
    }
    return true;
}

void resetEncoder()
{
    debug_msg("Sending reset command to Encoder");
    for (int i = 0; i < 2; i++)
    {
        Wire.beginTransmission(ENC[i]);
        Wire.write('r');
        Wire.endTransmission();
    }
    debug_msg("Encoder Reset Complete");
    readEncoder();
    debug_msg("Encoder_0 = " + String(encoderCount[0]) + "\tEncoder_1 = " + String(encoderCount[1]));
}

void debugEncoder()
{
    if (Serial.available())
    {
        char ch = Serial.read();
        resetEncoder();
    }
    readEncoder();
    if (encoderCount[0] != lastEncoderCount[0] || encoderCount[1] != lastEncoderCount[1])
    {
        Serial.print(encoderCount[0]);
        Serial.print("\t");
        Serial.println(encoderCount[1]);
        lastEncoderCount[0] = encoderCount[0];
        lastEncoderCount[1] = encoderCount[1];
    }
    delay(10);
}

#endif