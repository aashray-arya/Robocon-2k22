#ifndef PID_TUNER_H
#define PID_TUNER_H
#define PID_TUNER_DEBUG

#if ARDUINO > 22
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <HardwareSerial.h>



class PIDTuner
{
    HardwareSerial &port;
    bool *power;
    float *Kp;
    float *Ki;
    float *Kd;
    byte startMarker ;
    byte endMarker ;
    byte specialByte ;

    void serialFlush();
    bool readBytes(byte[], byte, char);
    void decodeData(byte[], byte, byte &, byte[]);
    void debug_msg(String);

public:
    // PIDTuner(byte *, float *, float *, float *, HardwareSerial &);
    PIDTuner(bool *_power, float *_Kp, float *_Ki, float *_Kd, HardwareSerial &newPort = Serial1) : port(newPort)
    {
        port.begin(115200);
        power = _power;
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        startMarker=254;
        endMarker=255;
        specialByte=253;
    }
    bool update();
};
#endif