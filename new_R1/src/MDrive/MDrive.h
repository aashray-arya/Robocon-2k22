#ifndef MDrive_H
#define MDrive_H
#include <Arduino.h>
#include "../IMU/IMU.h"
#include "../Encoder/Encoder.h"
#include "../Misc/Misc.h"
#include <PID_v1.h>
struct motor
{
    int D0;
    int D1;
    int PWM_PIN;
    constexpr motor(int x, int y, int z) : D0(x), D1(y), PWM_PIN(z)
    {
    }
};
struct comp
{
    float Vx;
    float Vy;
    float W;
};

enum PID_OBJ
{
    LIN = 0,
    ROT
}; //LIN AND ROT PID OBJECTS

static double input[2] = {0.0};
static double output[2] = {0.0};
static double setpoint[2] = {0.0};
static bool prevStopped = true;

PID linear(&input[LIN], &output[LIN], &setpoint[LIN], 0.0, 0.0, 0.0, DIRECT);
PID rotational(&input[ROT], &output[ROT], &setpoint[ROT], 0.0, 0.0, 0.0, DIRECT);

class Drive
{
    motor front1;
    motor front2;
    motor back3;
    motor back4;

    comp comp;
    double Kp[2];
    double Ki[2];
    double Kd[2];

    float f1PWM;
    float f2PWM;
    float b3PWM;
    float b4PWM;

    float f1PWM_cur;
    float f2PWM_cur;
    float b3PWM_cur;
    float b4PWM_cur;

    int MAX_PWM;

public:
    constexpr Drive(motor f1, motor f2, motor b3,motor b4, double *linCons, double *rotCons, int max = 150) : front1(f1), front2(f2), back3(b3),back4(b4),
    Kp{linCons[0], rotCons[0]}, Ki{linCons[1], rotCons[1]}, Kd{linCons[2], rotCons[2]},
    f1PWM(0.0), f2PWM(0.0), b3PWM(0.0), b4PWM(0.0),f1PWM_cur(0.0), f2PWM_cur(0.0), b3PWM_cur(0.0),b4PWM_cur(0.0),
    comp{0.0, 0.0, 0.0}, MAX_PWM(max)
    {
    }
    void initialize();
    void writeMotor(int, int, int,int);
    void rotation(int,int,int);
    void stopAll(bool HARD = true);
    //void lowPass(float);
    void move(int, float, float, float);
    void Rotate(int );
    void R90(int);
    void R_90(int);
};
void Drive::initialize()
{
    debug_msg("Initializing Drive");
    motor motors[] = {
        front1,
        front2,
        back3,
        back4};
        linear.SetSampleTime(50);
        rotational.SetSampleTime(50);
        linear.SetTunings(Kp[LIN], Ki[LIN], Kd[LIN]);
        rotational.SetTunings(Kp[ROT], Ki[ROT], Kd[ROT]);
        setpoint[0] = setpoint[1] = 0.0;
        for (motor i : motors)
        {
            pinMode(i.D0, OUTPUT);
            pinMode(i.D1, OUTPUT);
            pinMode(i.PWM_PIN, OUTPUT);
        }
        linear.SetOutputLimits(-360.0, 360.0);
        rotational.SetOutputLimits(-2.0, 2.0);
        linear.SetMode(AUTOMATIC);
        rotational.SetMode(AUTOMATIC);
        debug_msg("Drive Initialization Complete");
    }
    void Drive::writeMotor(const int f1, const int f2, const int b3 , const int b4)
    {
        if (f1 != -1)
        {
            digitalWrite(front1.D0, f1 > 0);
            digitalWrite(front1.D1, f1 < 0);
            analogWrite(front1.PWM_PIN, abs(f1));
        }
        if (f2 != -1)
        {
            digitalWrite(front2.D0, f2 > 0);
            digitalWrite(front2.D1, f2 < 0);
            analogWrite(front2.PWM_PIN, abs(f2));
        }
        if (b3 != -1)
        {
            digitalWrite(back3.D0, b3 > 0);
            digitalWrite(back3.D1, b3 < 0);
            analogWrite(back3.PWM_PIN, abs(b3));
        }
        if (b4 != -1)
        {
            digitalWrite(back4.D0, b4 > 0);
            digitalWrite(back4.D1, b4 < 0);
            analogWrite(back4.PWM_PIN, (abs(b4)+15));
        }
        String msg = ("F1 : " + String(f1) + "\tF2 : " + String(f2) + "\tB3: " + String(b3)+"\tB4: "+ String(b4));
        debug_msg(msg);
    }
    void Drive::stopAll(bool HARD = true)
    {
        constexpr float alpha = 0.0006;
        if (!prevStopped)
        {
            digitalWrite(front1.D0, HARD);
            digitalWrite(front1.D1, HARD);
            digitalWrite(front1.PWM_PIN, HARD);

            digitalWrite(front2.D0, HARD);
            digitalWrite(front2.D1, HARD);
            digitalWrite(front2.PWM_PIN, HARD);

            digitalWrite(back3.D0, HARD);
            digitalWrite(back3.D1, HARD);
            digitalWrite(back3.PWM_PIN, HARD);

            digitalWrite(back4.D0, HARD);
            digitalWrite(back4.D1, HARD);
            digitalWrite(back4.PWM_PIN, HARD);

            prevStopped = true;
            f1PWM = f2PWM = b3PWM = b4PWM =0;
            debug_msg("Stopped All Motors");
            String msg = ("F1 : " + String(f1PWM) + "\tF2 : " + String(f2PWM) + "\tB3: " + String(b3PWM)+"\tB4: "+ String(b4PWM));
            debug_msg(msg);
        }
    }
    void Drive::move(int R, float theta, float _head = 0.0, float w = 0)
    {
        static long startTime = millis();
        static float desiredAngle = getYaw();
        if (!prevStopped)
        {
            //readEncoder();
            if ((millis() - startTime) > THETA_SAMPLE_RATE)
            {
                if (!isnan(calculatedThetaXY))
                {
                    input[LIN] = calculatedThetaXY;
                    linear.Compute();
                // Serial.println(String(calculatedThetaXY) + "\t" + String(output[LIN]));
                }
                input[ROT] = angleDiff(getYaw(), desiredAngle);
                rotational.Compute();
            // debug_msg("INP : " + String(input[ROT]) + "\tOUT : " + String(output[ROT]));
            }
        }
        else
        {
            prevStopped = false;
            startTime = millis();
            R = map(constrain(R, 0, 100), 0, 100, 0, MAX_PWM);
            setpoint[ROT] = 0.0;
            setpoint[LIN] = theta;
        // desiredAngle = getYaw();
        }
        theta = toRadian(theta + output[LIN]);
        comp.Vx = cos(theta);
        comp.Vy = sin(theta);
        comp.W = w - output[ROT];

        float V1 = (comp.Vx + comp.Vy - 240*comp.W - 180*comp.W);
        float V2 = (comp.Vx - comp.Vy + 240*comp.W + 180*comp.W);
        float V3 = (comp.Vx - comp.Vy - 240*comp.W - 180*comp.W);
        float V4 = (comp.Vx + comp.Vy + 240*comp.W + 180*comp.W);
    //debug_msg( "\tYAW:" + String(getYaw()) + "\tSetpoint"+String (setpoint[ROT])+"\tVf:"+String(Vf)+"\tVl:"+String(Vl)+"\tVr:"+String(Vr));
        float maxComp = maxm(abs(V1), abs(V2), abs(V3),abs(V4));
        String msg = String(V1) + " " + String(V2) + " " + String(V3)+ " " + String(V4);
    //debug_msg(msg);
        if (maxComp == abs(V1))
        {
            f1PWM = R * (V1 / abs(V1));
            f2PWM = (float)R * (V2 / abs(V1));
            b3PWM = (float)R * (V3 / abs(V1));
            b4PWM = (float)R * (V4 / abs(V1));
        }
        else if (maxComp == abs(V2))
        {
            f1PWM = (float)R * (V1 / abs(V2));
            f2PWM = R * (V2 / abs(V2));
            b3PWM = (float)R * (V3 / abs(V2));
            b4PWM = (float)R * (V4 / abs(V2));
        }
        else if (maxComp == abs(V3))
        {
            f1PWM = (float)R * (V1 / abs(V3));
            f2PWM = (float)R * (V2 / abs(V3));
            b3PWM = R * (V3 / abs(V3));
            b4PWM = (float)R * (V4 / abs(V3));
        }
        else if (maxComp == abs(V4))
        {
            f1PWM = (float)R * (V1 / abs(V4));
            f2PWM = (float)R * (V2 / abs(V4));
            b3PWM = (float)R * (V3 / abs(V4));
            b4PWM = R * (V4 / abs(V4));
        }
        writeMotor(f1PWM, f2PWM, b3PWM, b4PWM);
    }
    void Drive::Rotate(int pwm)
    {
        if (pwm != -1)
        {
            prevStopped = false;
            digitalWrite(front1.D0, pwm<0);
            digitalWrite(front1.D1, pwm>0);
            analogWrite(front1.PWM_PIN, abs(pwm));

            digitalWrite(front2.D0, pwm<0);
            digitalWrite(front2.D1, pwm>0);
            analogWrite(front2.PWM_PIN, abs(pwm));

            digitalWrite(back3.D0, pwm>0);
            digitalWrite(back3.D1, pwm<0);
            analogWrite(back3.PWM_PIN, abs(pwm));

            digitalWrite(back4.D0, pwm>0);
            digitalWrite(back4.D1, pwm<0);
            analogWrite(back4.PWM_PIN, abs(pwm));

            debug_msg("Speed of Rotation:\t" + String(pwm));

        }
    }
    void Drive::R90(int p)
    {
      int x1=getYaw()-90;
      if(x1 < 0){
        x1+=360;
        while(getYaw() <x1)
        {       
            Drive::Rotate(p);
        }
    }                      
        else
        {
            while(getYaw()>x1)
            { 
                Drive::Rotate(p);
            }
        }
        Drive::stopAll();                 
    }
    void Drive::R_90(int p1)
    {
        int x2=getYaw()+90;
        if(x2 > 360){
            x2-=360;
            while(getYaw()>x2)
            { 
                Drive::Rotate(-1*p1);
            }
        }                      
        else
        {
            while(getYaw()<x2)
            { 
                Drive::Rotate(-1*p1);
            }
        }
        Drive::stopAll();
    }
#endif
