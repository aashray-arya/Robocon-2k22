#ifndef HolonomicDrive_H
#define HolonomicDrive_H
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

class HolonomicDrive
{
    motor front;
    motor left;
    motor right;

    comp comp;
    double Kp[2];
    double Ki[2];
    double Kd[2];

    float frontPWM;
    float leftPWM;
    float rightPWM;

    float frontPWM_cur;
    float leftPWM_cur;
    float rightPWM_cur;

    int MAX_PWM;

  public:
    constexpr HolonomicDrive(motor f, motor l, motor r, double *linCons, double *rotCons, int max = 150) : front(f), left(l), right(r),
                                                                                                           Kp{linCons[0], rotCons[0]}, Ki{linCons[1], rotCons[1]}, Kd{linCons[2], rotCons[2]},
                                                                                                           frontPWM(0.0), leftPWM(0.0), rightPWM(0.0), frontPWM_cur(0.0), leftPWM_cur(0.0), rightPWM_cur(0.0),
                                                                                                           comp{0.0, 0.0, 0.0}, MAX_PWM(max)
    {
    }
    void initialize();
    void writeMotor(int, int, int);
    void rotation(int,int,int);
    void stopAll(bool HARD = true);
    //void lowPass(float);
    void move(int, float, float, float);
    void Rotate_AK(int );
};

void HolonomicDrive::initialize()
{
    debug_msg("Initializing Drive");
    motor motors[] = {
        front,
        left,
        right};
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

void HolonomicDrive::writeMotor(const int f, const int l, const int r)
{
    if (f != -1)
    {
        digitalWrite(front.D0, f > 0);
        digitalWrite(front.D1, f < 0);
        analogWrite(front.PWM_PIN, abs(f));
    }
    if (l != -1)
    {
        digitalWrite(left.D0, l > 0);
        digitalWrite(left.D1, l < 0);
        analogWrite(left.PWM_PIN, abs(l));
    }
    if (r != -1)
    {
        digitalWrite(right.D0, r > 0);
        digitalWrite(right.D1, r < 0);
        analogWrite(right.PWM_PIN, abs(r));
    }
    String msg = ("F : " + String(f) + "\tL : " + String(l) + "\tR : " + String(r));
    debug_msg(msg);
}

void HolonomicDrive::stopAll(bool HARD = true)
{
    constexpr float alpha = 0.0006;
    if (!prevStopped)
    {
        // float frontPWM_LP = frontPWM;
        // float leftPWM_LP = leftPWM;
        // float rightPWM_LP = rightPWM;
        // Serial.println(String(frontPWM_LP) + "\t" + String(leftPWM_LP) + "\t" + String(rightPWM_LP));
        // while ((int)frontPWM_LP != 0 || (int)leftPWM != 0 || (int)rightPWM != 0)
        // {
        //     frontPWM_LP -= frontPWM_LP * alpha;
        //     leftPWM_LP -= leftPWM_LP * alpha;
        //     rightPWM_LP -= rightPWM_LP * alpha;
        //     writeMotor(frontPWM_LP, leftPWM_LP, rightPWM_LP);
        //     delay(10);
        // }
        digitalWrite(front.D0, HARD);
        digitalWrite(front.D1, HARD);
        digitalWrite(front.PWM_PIN, HARD);

        digitalWrite(left.D0, HARD);
        digitalWrite(left.D1, HARD);
        digitalWrite(left.PWM_PIN, HARD);

        digitalWrite(right.D0, HARD);
        digitalWrite(right.D1, HARD);
        digitalWrite(right.PWM_PIN, HARD);

        prevStopped = true;
        frontPWM = leftPWM = rightPWM = 0;
        debug_msg("Stopped All Motors");
    }
}

//void HolonomicDrive : lowPass(float alpha)
//{
//    frontPWM_cur += alpha * (frontPWM - frontPWM_cur);
//    leftPWM_cur += alpha * (leftPWM - leftPWM_cur);
//    rightPWM_cur += alpha * (rightPWM - rightPWM_cur);
//    writeMotor(frontPWM_cur, leftPWM_cur, rightPWM_cur);
//}

void HolonomicDrive::move(int R, float theta, float _head = 0.0, float w = 0)
{
    static long startTime = millis();
    static float desiredAngle = getYaw();
    if (!prevStopped)
    {
        readEncoder();
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

    float Vf = (comp.Vx + comp.W);
    float Vl = -0.5 * comp.Vx + 0.867 * comp.Vy + comp.W;
    float Vr = -0.5 * comp.Vx - 0.867 * comp.Vy + comp.W;
    //debug_msg( "\tYAW:" + String(getYaw()) + "\tSetpoint"+String (setpoint[ROT])+"\tVf:"+String(Vf)+"\tVl:"+String(Vl)+"\tVr:"+String(Vr));
    float maxComp = maxm(abs(Vf), abs(Vl), abs(Vr));
    String msg = String(Vf) + " " + String(Vl) + " " + String(Vr);
    // debug_msg(msg);
    if (maxComp == abs(Vf))
    {
        frontPWM = R * (Vf / abs(Vf));
        leftPWM = (float)R * (Vl / abs(Vf));
        rightPWM = (float)R * (Vr / abs(Vf));
    }
    else if (maxComp == abs(Vl))
    {
        frontPWM = (float)R * (Vf / abs(Vl));
        leftPWM = R * (Vl / abs(Vl));
        rightPWM = (float)R * (Vr / abs(Vl));
    }
    else if (maxComp == abs(Vr))
    {
        frontPWM = (float)R * (Vf / abs(Vr));
        leftPWM = (float)R * (Vl / abs(Vr));
        rightPWM = R * (Vr / abs(Vr));
    }
    writeMotor(frontPWM, leftPWM, rightPWM);
}

void HolonomicDrive::Rotate_AK(int pwm)
{
    if (pwm != -1)
    {
        prevStopped = false;
        digitalWrite(front.D0, pwm>0);
        digitalWrite(front.D1, pwm<0);
        analogWrite(front.PWM_PIN, abs(pwm));

        digitalWrite(left.D0, pwm>0);
        digitalWrite(left.D1, pwm<0);
        analogWrite(left.PWM_PIN, abs(pwm));

        digitalWrite(right.D0, pwm>0);
        digitalWrite(right.D1, pwm<0);
        analogWrite(right.PWM_PIN, abs(pwm));

        debug_msg("Speed of Rotation:\t" + String(pwm));

    }
}
#endif
