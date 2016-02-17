
/* astar stuff */
#include <AStar32U4.h>
#include <FastGPIO.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include "AStarEncoders.h"
/* ros stuff */
#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>


ros::NodeHandle  nh;

std_msgs::Int16 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);

std_msgs::Int16 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);

std_msgs::Float32 lwheelVelocityMsg;
ros::Publisher lwheelVelocityPub("lwheel_velocity", &lwheelVelocityMsg);

std_msgs::Float32 rwheelVelocityMsg;
ros::Publisher rwheelVelocityPub("rwheel_velocity", &rwheelVelocityMsg);
/*
void lwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> lwheelTargetSub("lwheel_vtarget", &lwheelTargetCallback);

void rwheelTargetCallback(const std_msgs::Float32& cmdMsg);
ros::Subscriber<std_msgs::Float32> rwheelTargetSub("rwheel_vtarget", &rwheelTargetCallback);
*/
AStarEncoders encoders;
AStar32U4Motors motors;

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?
int last = 1;
char report[150];
int speed = 0;

void setup()
{
  Serial.begin(9600);
  // Uncomment to flip a motor's direction:
  //motors.flipM1(true);
  motors.flipM2(true);


 
  
}




void loop() {
  // put your main code here, to run repeatedly:
  static uint16_t lastMotorTime;
  static uint8_t lastTime;
    static uint16_t lastDisplayTime;
// display data ever 500ms
  if ((millis() - lastDisplayTime) >= 500)
  {
    lastDisplayTime = millis();

     int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();
    // Send the information to the serial monitor also.
    snprintf_P(report, sizeof(report),
        PSTR("left = %6d right =%6d %3d  %6d %3d"),
           countsLeft, countsRight, errorLeft, last);  
    Serial.println(report);
  }
 //last = 0;
    if ( (uint16_t)(millis() - lastMotorTime) >= 2000)
    {
      lastMotorTime = millis();

           switch (last){
            case 1:
            
              break;
            case 2:
            ledYellow(1);
            motors.setM1Speed(150);
            motors.setM2Speed(150);
              break;
            case 3:
            ledYellow(1);
            motors.setM1Speed(-150);
            motors.setM2Speed(-150);
              break;
            case 4:
            ledYellow(1);
            motors.setM1Speed(0);
            motors.setM2Speed(0);
              break;
            case 6:
            ledYellow(1);
            motors.setM1Speed(100);
            motors.setM2Speed(-100);
              break;
   
              case 7:
            ledYellow(1);
            motors.setM1Speed(0);
            motors.setM2Speed(0);
              break;
            case 9:
            ledYellow(1);
            motors.setM1Speed(150);
            motors.setM2Speed(150);
              break;
            case 10:
            ledYellow(1);
            motors.setM1Speed(-150);
            motors.setM2Speed(-150);
              break;
            case 11:
            ledYellow(1);
            motors.setM1Speed(0);
            motors.setM2Speed(0);
              break;
            case 13:
            ledYellow(1);
            motors.setM1Speed(-100);
            motors.setM2Speed(100);
              break;
   
              case 14:
            ledYellow(1);
            motors.setM1Speed(0);
            motors.setM2Speed(0);
              break;

            case 18:
              last = 0;
              break;
              
            default: 
            // if nothing else matches, do the default
            // default is optional
            break;
           }
      last++;
    
    }





  
}
