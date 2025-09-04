/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=100, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double in;
void setup()
{
  //initialize the variables we're linked to
  Serial.begin(115200);
  Input = in;
  Setpoint = 5;

  //turn the PID on
  myPID.SetTunings(50, 0, 0);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
}

void loop()
{
  in+=0.1;
  Input = in;
  myPID.Compute();
  Serial.println(Output);
  delay(500);
}


