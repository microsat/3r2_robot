#include <AStar32U4.h>
#include <FastGPIO.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#define LEFT_XOR   8
#define LEFT_B     15
#define RIGHT_XOR  7
#define RIGHT_B    16

AStar32U4Motors motors;

static volatile bool lastLeftA;
static volatile bool lastLeftB;
static volatile bool lastRightA;
static volatile bool lastRightB;

static volatile bool errorLeft;
static volatile bool errorRight;

static volatile uint16_t countLeft;
static volatile uint16_t countRight;


int last = 1;
char report[150];

ISR(PCINT0_vect)
{
    bool newLeftB = FastGPIO::Pin<LEFT_B>::isInputHigh();
    bool newLeftA = FastGPIO::Pin<LEFT_XOR>::isInputHigh() ^ newLeftB;

    countLeft += (newLeftA ^ lastLeftB) - (lastLeftA ^ newLeftB);
//countLeft++;
    if((lastLeftA ^ newLeftA) & (lastLeftB ^ newLeftB))
    {
        errorLeft = true;
    }

    lastLeftA = newLeftA;
    lastLeftB = newLeftB;
}

static void rightISR()
{
    bool newRightB = FastGPIO::Pin<RIGHT_B>::isInputHigh();
    bool newRightA = FastGPIO::Pin<RIGHT_XOR>::isInputHigh() ^ newRightB;

    countRight += (newRightA ^ lastRightB) - (lastRightA ^ newRightB);
//countRight++;
    if((lastRightA ^ newRightA) & (lastRightB ^ newRightB))
    {
        errorLeft = true;
    }

    lastRightA = newRightA;
    lastRightB = newRightB;
}

void setup()
{
  Serial.begin(9600);
  // Uncomment to flip a motor's direction:
  //motors.flipM1(true);
  //motors.flipM2(true);
  // Enable pin-change interrupt on PB1 for left encoder, and disable other
    // pin-change interrupts.
    PCICR = (1 << PCIE0);
    PCMSK0 = (1 << PCINT4);
    PCIFR = (1 << PCIF0);  // Clear its interrupt flag by writing a 1.


   // Enable interrupt on PE6 for the right encoder.  We use attachInterrupt
    // instead of defining ISR(INT6_vect) ourselves so that this class will be
    // compatible with other code that uses attachInterrupt.
    attachInterrupt(4, rightISR, CHANGE);

       lastLeftB = FastGPIO::Pin<LEFT_B>::isInputHigh();
    lastLeftA = FastGPIO::Pin<LEFT_XOR>::isInputHigh() ^ lastLeftB;
    countLeft = 0;
    errorLeft = 0;

    lastRightB = FastGPIO::Pin<RIGHT_B>::isInputHigh();
    lastRightA = FastGPIO::Pin<RIGHT_XOR>::isInputHigh() ^ lastRightB;
    countRight = 0;
    errorRight = 0;
  
}




void loop() {
  // put your main code here, to run repeatedly:

    static uint16_t lastDisplayTime;
// display data ever 500ms
  if ((millis() - lastDisplayTime) >= 500)
  {
    lastDisplayTime = millis();
 
    // Send the information to the serial monitor also.
    snprintf_P(report, sizeof(report),
        PSTR("left = %6d right =%6d %3d  %6d %3d"),
        countLeft, countRight, errorLeft, errorRight);
    Serial.println(report);
  }
}
