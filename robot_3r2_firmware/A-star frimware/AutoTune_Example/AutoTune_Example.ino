#include <AStar32U4.h>
#include <AStar32U4Buttons.h>
#include <AStar32U4Buzzer.h>
#include <AStar32U4LCD.h>
#include <AStar32U4Motors.h>
#include <FastGPIO.h>
#include <PololuBuzzer.h>
#include <PololuHD44780.h>
#include <Pushbutton.h>
#include <SPIPause.h>
#include <USBPause.h>

#include <AStarEncoders.h>

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

AStarEncoders encoders;
AStar32U4Motors motors;

byte ATuneModeRemember=2;
double input=80, output=60, setpoint=50;
double kp=2,ki=0.5,kd=2;
double current,last;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=100, aTuneNoise=1, aTuneStartValue=150;
unsigned int aTuneLookBack=5;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

//set to false to connect to the real world
//boolean useSimulation = true;
boolean useSimulation = false;

void setup()
{
  
  // Uncomment to flip a motor's direction:
  motors.flipM1(true);
  motors.flipM2(true);

  myPID.SetOutputLimits(-400,400);
  myPID.SetSampleTime(20);
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(57600);
 current=0;
 last=current;
}

void loop()
{

  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    //input = analogRead(0);
    input = encoders.getCountsLeft();
   
   
    
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else
  {
     //analogWrite(0,output);
     motors.setM1Speed(output) ;
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  int error = (int)encoders.checkErrorLeft();
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  Serial.print("error: ");Serial.print(error); Serial.print(" ");
  
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}
