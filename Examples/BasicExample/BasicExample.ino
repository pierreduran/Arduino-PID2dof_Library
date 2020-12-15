//Validation code or example of PID 2DoF library

//Libraries
#include <PID_2dof.h>

//Define the pins of Arduino
#define PIN_INPUT A0
#define PIN_OUTPUT 3

//Define variables
double input, setpoint, output;

//Define the tuning parameters
double Kp=2, Ki=5, Kd=1, beta=1, gamma=0, alpha=0;

//Define the class and some variables to the process
PID_2dof myPID(&input, &setpoint, &output, Kp, Ki, Kd, beta, gamma, alpha);

void setup() {
  //Initiate Serial communication
  Serial.begin(9600);

  //Define the input
  input = analogRead(PIN_INPUT);

  //Define the setpoint
  setpoint=1023;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  //Set the output
  //myPID.SetOutput(255);
}

void loop() {
  input = analogRead(PIN_INPUT);
  myPID.ComputePID();
  analogWrite(PIN_OUTPUT, output);

  //Serial print
  Serial.print("Salida = ");
  Serial.println(output);	
}
