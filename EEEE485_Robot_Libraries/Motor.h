
#pragma once
#ifndef Motor_Control_h
#define Motor_Control_h

#include "Arduino.h"

class Motor {
public:
	Motor(int dir_pin, int pwm_pin, int enbl_pin);
	double getVel();
	bool setVel(double speed, int direction = 1);
	void setVel_basic(double speed, int direction = 1);
	void setCurrentVel(double current_vel);
	void setPID(double p, double i, double d);

private:
	int DIR_PIN;
	int PWM_PIN;
	int ENBL_PIN;

	//PID vars
	double curr_err;
	double delta_err;
	double old_err;

	double pid_p = 2;
	double pid_i  = 0.2;
	double pid_d = -0.0; //-0.5;

	double set_vel; //what the motor is currently set at (may not be target value)
	double current_vel; //updates periodically and represents speed of the motor as ticks, rpm, or whatever

	double getVelDelta(double target, double tolerance = 0.1); //10% default tolerance

	static void calcVel_tisr(void);
};

#endif
