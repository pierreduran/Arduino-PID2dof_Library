/************************************************************************
	PID_2dof - Library to be used in the creation of a 2dof pid.
	Created by Pierre Durán Guzmán <pierre.duran@ucr.ac.cr>
	Version 1.0
************************************************************************/

#include <PID_2dof.h>

//Constructor
PID_2dof::PID_2dof(double *input, double *setpoint, double *output, double Kp, 
double Ki, double Kd, double beta, double gamma, double alpha) 
{

//variables
	_input = input;
	_setpoint = setpoint;
	_output = output;

//Tuning parameters of the controller
	PID_2dof::setParams(Kp, Ki, Kd, beta, gamma, alpha);

//Initially the controller is in manual
	AutoMode = false;
 
//Default limits output Arduino
	PID_2dof::Limits(0, 255);	
	
//Default sampletime in 100ms
	_SampleTime = 100;	
}

/*Calculate() the PID_2dof if the control is in automatic 
returns true if the output is compute but if the controller
is on manual, returns false and set the output by the user
*/

double PID_2dof::ComputePID()
{
	if(!AutoMode) return *_output;
		//Beging the P calculation
		unsigned long current_time = millis();
		
		//Going to compute all the PID_2dof
		//Calculate the proportional
		double error_p = (_beta * (*_setpoint)) - *_input;
		double proportional = _Kp * error_p;
	
		//Calculate the integer
		double error = *_setpoint - *_input;
		integer += _Ki * _SampleTime * error;
		//anti-windup configuration
		if (integer > _max) integer = _max;
		else if (integer < _min) integer = _min;
	
		//Calculate the derivative
		double delta_input = *_input - last_input;
		double K_filter = _alpha * (_Kd / _Kp);
		double derivative = ((K_filter / (_SampleTime + K_filter)) * derivative) +
		((_Kd / (_SampleTime + K_filter)) * (_gamma * (*_setpoint) - delta_input));
	
		//Determinate the PID output (u(k))
		double PID_2dof_out = proportional + integer + derivative;
		//Anti-output saturation and output of the controller
		*_output = constrain(PID_2dof_out, _min, _max);
	
		// Update values to current values
		last_time = current_time;
		last_input = *_input;
		
		//return de current output compute
		return *_output;
}

//Set the output if the controller is in manual
void PID_2dof::SetOutput(double MyOutput) 
{
	_MyOutput = MyOutput;
	if(_MyOutput > _max) _MyOutput = _max;
	else if(_MyOutput < _min) _MyOutput = _min;
	*_output = _MyOutput;
}

//Set tuning parameters
void PID_2dof::setParams(double Kp, double Ki, double Kd, double beta, 
double gamma, double alpha) 
{
	if (Kp< 0 || Ki< 0 || Kd< 0 || beta< 0 || gamma< 0 || alpha< 0 ) return;
		_Kp = Kp;
		_Ki = Ki;
		_Kd = Kd;
		_beta = beta;
		_gamma = gamma;
		_alpha = alpha;
}

//Set the limits of the output
void PID_2dof::Limits(double min, double max) 
{
	_min = min;
	_max = max;
}

// Set the sampleTime
void PID_2dof::SetSampleTime(unsigned long SampleTime)
{
	_SampleTime = SampleTime;
}

//Set Mode manual or automatic
void PID_2dof::SetMode(int Mode) 
{
	bool Auto = (Mode == AUTOMATIC);
	if(Auto && !AutoMode) 
	{
		PID_2dof::Init();
	}
	AutoMode = Auto;
}

//Funtion to make a bumpless transfer
void PID_2dof::Init() 
{
	last_input = *_input;
	integer = *_output;
	if (integer > _max) integer = _max;
	else if (integer < _min) integer = _min;
}