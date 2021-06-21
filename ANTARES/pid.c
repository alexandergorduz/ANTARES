#include <avr/io.h>
#include "receiver.h"
#include "pid.h"

#define controlMax 400

#define anglePropGain 4.0

#define rollRatePropGain 2.3
#define rollRateIntGain 0.018
#define rollRateDerGain 28.0

#define pitchRatePropGain 2.3
#define pitchRateIntGain 0.018
#define pitchRateDerGain 30.0

#define yawRatePropGain 1.3
#define yawRateIntGain 0.08
#define yawRateDerGain 0.0

uint8_t rollClamp = 0;
uint8_t pitchClamp = 0;
uint8_t yawClamp = 0;

float error;
float rollRateSetp, pitchRateSetp, yawRateSetp;
float rollControl, rollRateLastErr, rollRateInt;
float pitchControl, pitchRateLastErr, pitchRateInt;
float yawControl, yawRateLastErr, yawRateInt;
float throttleControl;

float rollSetp, pitchSetp, yawSetp;
int recThrottle;

extern float roll, pitch;
extern float rollRate, pitchRate, yawRate;

void calcPID()
{
	error = rollSetp - roll;
	rollRateSetp = error * anglePropGain;
	error = rollRateSetp - rollRate;
	if (rollClamp == 0 && recThrottle >= 1100) rollRateInt += error * rollRateIntGain;
	if (rollRateInt > controlMax) rollRateInt = controlMax;
	if (rollRateInt < -controlMax) rollRateInt = -controlMax;
	rollControl = error * rollRatePropGain + rollRateInt + (error - rollRateLastErr) * rollRateDerGain;
	if ((rollControl > controlMax || rollControl < -controlMax) && (error * rollControl > 0)) rollClamp = 1;
	else rollClamp = 0;
	if (rollControl > controlMax) rollControl = controlMax;
	if (rollControl < -controlMax) rollControl = -controlMax;
	rollRateLastErr = error;
	
	error = pitchSetp - pitch;
	pitchRateSetp = error * anglePropGain;
	error = pitchRateSetp - pitchRate;
	if (pitchClamp == 0 && recThrottle >= 1100) pitchRateInt += error * pitchRateIntGain;
	if (pitchRateInt > controlMax) pitchRateInt = controlMax;
	if (pitchRateInt < -controlMax) pitchRateInt = -controlMax;
	pitchControl = error * pitchRatePropGain + pitchRateInt + (error - pitchRateLastErr) * pitchRateDerGain;
	if ((pitchControl > controlMax || pitchControl < -controlMax) && (error * pitchControl > 0)) pitchClamp = 1;
	else pitchClamp = 0;
	if (pitchControl > controlMax) pitchControl = controlMax;
	if (pitchControl < -controlMax) pitchControl = -controlMax;
	pitchRateLastErr = error;
	
	error = yawSetp;
	yawRateSetp = error * anglePropGain;
	error = yawRateSetp - yawRate;
	if (yawClamp == 0 && recThrottle >= 1100) yawRateInt += error * yawRateIntGain;
	if (yawRateInt > controlMax) yawRateInt = controlMax;
	if (yawRateInt < -controlMax) yawRateInt = -controlMax;
	yawControl = error * yawRatePropGain + yawRateInt + (error - yawRateLastErr) * yawRateDerGain;
	if ((yawControl > controlMax || yawControl < -controlMax) && (error * yawControl > 0)) yawClamp = 1;
	else yawClamp = 0;
	if (yawControl > controlMax) yawControl = controlMax;
	if (yawControl < -controlMax) yawControl = -controlMax;
	yawRateLastErr = error;
	
	throttleControl = recThrottle;
}