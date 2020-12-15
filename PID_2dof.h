/************************************************************************
	PID_2dof - Library to be used in the creation of a 2dof pid.
	Created by Pierre Durán Guzmán <pierre.duran@ucr.ac.cr>
	Version 1.0
************************************************************************/

#ifndef PID_2dof_h	//"Guard" directive
#define PID_2dof_h

#include <Arduino.h>	//"include" directive

class PID_2dof
{
	public:
	//Constructor: takes pointer inputs for control variales, so they are 
	//updated automatically
	PID_2dof(double *input, double *setpoint, double *output, double Kp, 
	double Ki, double Kd, double beta, double gamma, double alpha);
	
	//Sets parameters of the PID
	void setParams(double Kp, double Ki, double Kd, double beta, 
	double gamma, double alpha);
	
	//Sets the mode manual or automatic 
	void SetMode(int Mode);
	//Constants used in this function
	#define AUTOMATIC 1
	#define MANUAL 0
	
	//Set the output controller if the controller is Manual
	void SetOutput(double MyOutput);
	
	//Compute the PID 2DoF calculation. Need to be use on the 
	//loop() 
	double ComputePID();
	
	//Determinate the limits of the output 
	void Limits(double min, double max);
	
	//Specify the the frequency or the sampling time of the PID 
	//in milliseconds
	void SetSampleTime(unsigned long SampleTime);
  
	private:
	//variables double
	double *_input, *_setpoint, *_output;
	double _Kp, _Kd, _Ki, _beta, _gamma, _alpha;
	double _MyOutput;
	double _min, _max;
	double last_input;
	double integer;
	
	//variable sampling time
	unsigned long _SampleTime;
	
	//the last time
	unsigned long last_time;
	
	//For SetMode
	bool AutoMode;
	void Init();
	
}; //class PID_2dof

#endif