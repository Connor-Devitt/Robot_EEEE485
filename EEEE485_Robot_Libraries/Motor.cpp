#include "Motor.h"

Motor::Motor(int dir_pin, int pwm_pin, int enbl_pin) {
	DIR_PIN = dir_pin;
	PWM_PIN = pwm_pin;
	ENBL_PIN = enbl_pin;

	this->pid_p = pid_p;
	this->pid_i = pid_i;
	this->pid_d = pid_d;

	this->curr_err = 0;
	this->delta_err = 0;
	this->old_err = 0;

	this->set_vel = 0;
	this->current_vel = 0;

	//Setup pins for use
	pinMode(DIR_PIN, OUTPUT);
	pinMode(PWM_PIN, OUTPUT);
	pinMode(ENBL_PIN, OUTPUT);

	//Enable this motor by default
	digitalWrite(ENBL_PIN, HIGH);
}

double Motor::getVel() {
	return current_vel;
}

double Motor::getVelDelta(double target, double tolerance) {
	double output_val = 0;

	curr_err = abs(getVel()) - target;

	if (abs(curr_err/target) <= tolerance) {
		return 0;
	}

	delta_err = (old_err + curr_err) / 2.0;

	output_val += pid_p * curr_err;
	output_val += pid_i * old_err;
	output_val += pid_d * delta_err;

	//  old_err_sum += curr_err;
	old_err = curr_err;

	return -output_val;
}

bool Motor::setVel(double speed, int direction) {
	set_vel += getVelDelta(speed);
	if(set_vel > 255) {
		set_vel = 255;
	}
	else if(set_vel <= 0) {
		set_vel = 0;
	}

	analogWrite(PWM_PIN, set_vel);

	switch(direction) {
		case 1:
			digitalWrite(DIR_PIN, HIGH);
			break;
		case -1:
			digitalWrite(DIR_PIN, LOW);
			break;
		default:
			digitalWrite(DIR_PIN, HIGH);
			break;
	}

	if(abs((direction * current_vel - speed)) <= 1) {
		return true;
	}
	else return false;
}

void Motor::setVel_basic(double speed, int direction) {
	if(speed <= 0) {
		analogWrite(PWM_PIN, 0);
		// digitalWrite(ENBL_PIN, LOW);
		//Reset PID because robot gets angry
		curr_err = 0;
		delta_err = 0;
		old_err = 0;

		setCurrentVel(0);

	}
	else {
		// digitalWrite(ENBL_PIN, HIGH);

		analogWrite(PWM_PIN, speed);

		switch(direction) {
			case 1:
				digitalWrite(DIR_PIN, HIGH);
				break;
			case -1:
				digitalWrite(DIR_PIN, LOW);
				break;
			default:
				digitalWrite(DIR_PIN, HIGH);
				break;
		}
	}

}

void Motor::setCurrentVel(double current_vel) {
	this->current_vel = current_vel;
}
