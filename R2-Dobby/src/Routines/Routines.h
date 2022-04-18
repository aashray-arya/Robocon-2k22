#include <Arduino.h>
#include "../Encoder/Encoder.h"
#include "../IMU/IMU.h"
#include "../HolonomicDrive/HolonomicDrive.h"
#include <PID_v1.h>

constexpr motor front(5, 6, 4);
constexpr motor left(10, 9, 8); 
constexpr motor right(13, 12, 11);  

constexpr int MAX_SPEED = 200;

constexpr double lConst[] = {0.0, 0.0, 0.0};
constexpr double rConst[] = {0.0, 0.0, 0.0};

HolonomicDrive botAuto(front, left, right, lConst, rConst, MAX_SPEED); 

class Routines
{

  public:
	int curTar;
	int running;

	


	
}