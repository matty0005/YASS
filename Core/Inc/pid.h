/*
 * pid.h
 *
 *  Created on: Dec 19, 2021
 *      Author: Matt Gilpin
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float tau;
	float min;
	float max;

	float T;

	float integrator;
	float prevErr;
	float differntiator;
	float prevMeas;

	float out;
} PIDController;


void PID_init(PIDController *pid);
float PID_update(PIDController *pid, float setpoint, float measurement);


#endif /* INC_PID_H_ */
