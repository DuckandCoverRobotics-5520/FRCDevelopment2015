#ifndef PID_H_
#define PID_H_
/*
Credit where credit is due:
Dave Hawes -- introducing me to the concept.
David Cohen-- showing me the real world application..
Sue Shields-- helping me find the "dependant" form of I, this greatly simplified the process.
David Shields-- demonstrating how to tune The PID controller.
*/

//Define parameters
#define Kp 0.03//tuning for p
#define Ki 0.003//tuning for i
#define Kd 0.0//tuning for d
int Tolerance =30;//how close is close enough?
#define dt 0.005 //duration of loop
float max_out= 0.25; //output limits
float min_out =-0.25;//^^^^^^^^^^^^
float PID(float setpoint, float ProcessValue)
{
	static float init_error= 0;
	static float integral=0;
	float error;
	float derivative;
	float PIDoutput;
//Caculate P
		error = setpoint -ProcessValue;
		//stop integration if error too small
		if(std::abs(error)> Tolerance)
		{
//Calculate I
		integral =integral+ error*dt;
		}
//Calculate D
		derivative= (error -init_error)/dt;
//COMPUTE PID
		PIDoutput=Kp*error + Ki*integral + Kd*derivative;
		//limits PIDoutput to ensure not out of range
		if(PIDoutput< min_out)
		{
		PIDoutput= min_out;
		}
		else if(PIDoutput> max_out)
		{
		PIDoutput= max_out;
		}
	//Update error, this program has to be run in a loop so that error can be corected over time
	init_error= error;
	return PIDoutput;
}
#endif
