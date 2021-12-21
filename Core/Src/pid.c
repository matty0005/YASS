/*
 * pid.c
 *
 *  Created on: Dec 19, 2021
 *      Author: Matt Gilpin
 */

#include "pid.h"

void PID_init(PIDController *pid) {
	pid->integrator = 0;
	pid->prevErr = 0;

	pid->differntiator = 0;
	pid->prevMeas = 0;
	pid->out = 0;

}


float PID_update(PIDController *pid, float setpoint, float measurement) {
	// Error
	float error = setpoint - measurement;

	float proportional = pid->Kp * error;

	pid->integrator = pid->integrator + (pid->Ki * pid->T / 2 ) * (error + pid->prevErr);



	// Anti-windup
	float limMinInt, limMaxInt;

	if (pid->max > proportional) {
		limMaxInt = pid->max - proportional;
	} else {
		limMaxInt = 0;
	}

	if (pid->min < proportional) {
		limMinInt = pid->min - proportional;
	} else {
		limMinInt = 0;
	}


	// Clamp integrator

	if (pid->integrator > limMaxInt) {
		pid->integrator = limMaxInt;
	} else if (pid->integrator < limMinInt) {
		pid->integrator = limMinInt;
	}



	float diffTerm1 = (2 * pid->tau - pid->T) / (2 * pid->tau + pid->T) * pid->differntiator;

	pid->differntiator = 2 * pid->Kd * (measurement - pid->prevMeas) + diffTerm1;


	// Sum of terms
	pid->out = proportional + pid->integrator + pid->differntiator;

	if (pid->out > pid->max) {
		pid->out = pid->max;
	} else if (pid->out < pid->min) {
		pid->out = pid->min;
	}


	pid->prevErr = error;
	pid->prevMeas = measurement;

	return pid->out;

}
