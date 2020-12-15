//Validation code or example of PID 2DoF library

//Libraries
#include <PID_2dof.h>
#include <TaskScheduler.h>

//Define the pins of Arduino
#define PIN_INPUT A0
#define PIN_OUTPUT 3
int LED = 13;

//Define variables
double input, setpoint, output;

//Define the tuning parameters
double Kp=2.79, Ki=0.16, Kd=7.25, beta=0.0167, gamma=0.00015, alpha=0.034;

//Define the class and some variables to the process
PID_2dof myPID(&input, &setpoint, &output, Kp, Ki, Kd, beta, gamma, alpha);

//Use Scheduler
Scheduler runner;

//Function to run the Compute PID
void Compute();
void Blink();

//Create the task, to execute the Compute Function
Task PIDCompute(100, TASK_FOREVER, &Compute);
Task Blink13(3000, TASK_FOREVER, &Blink);

void setup() {
  //Initiate Serial communication
  Serial.begin(9600);

  //Define the input
  input = analogRead(PIN_INPUT);

  //initialize pin LED
  pinMode(LED, OUTPUT);

  //Define the setpoint
  setpoint=512;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  //Add the task to Scheduler
  runner.addTask(PIDCompute);
  runner.addTask(Blink13);

  //Enable the task
  PIDCompute.enable();
  Blink13.enable();
  
  //Set the output
  //myPID.SetOutput(255);
}

void loop() {
  runner.execute();
}

void Compute() {
  input = analogRead(PIN_INPUT);
  myPID.ComputePID();
  analogWrite(PIN_OUTPUT, output);

  //Serial print
  Serial.print("Salida = ");
  Serial.println(output);
}

void Blink() {
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
}
