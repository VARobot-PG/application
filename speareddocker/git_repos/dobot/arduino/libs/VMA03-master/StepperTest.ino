/*
vma03Stepper library by Michael Hoefeler
for Velleman VMA03 motor driver
based on the great work of epsilon99 and toby-sk found at https://gist.github.com/Epsilon99/2757e52e06a7d2d50221

Version 0.1, 05/16/2015 
*/

// include the library to your code
#include <VMA03Stepper.h>

/*
vma03Stepper( byte pwmaPin, byte diraPin, 
              byte pwmbPin, byte dirbPin, 
              int stepsPerRevolution, 
              int speed, 
              int speedOffset);
              
 pwmaPin, diraPin, pwmbPin, dirbPin 
   = the pins configured on your VMA03                 

 stepsPerRevolution 
   = controls the vma03Stepper.degree()and vma03Stepper.setDegree() methods, 
   usually the steps your motor takes for a full turn of 360 degrees,
   but if there is a gearbox, a v-belt, a tooth-belt or some gear wheels behind 
   e.g. with a ratio of 1:5 it is for a 200 steps per full turn motor = 1000
 
 speed 
   = delay between each half-step measured in microseconds, higher values means less speed!
 
 speedOffset 
   = minimum speed to prevent the motor from turning to the next half step without having the last done
   if speed is less than speedOffset it will be set to speedOffset
-------------------------------------------------------------------------------------------
vma03Stepper class methods

  setSpeed(int speed)
    = delay between each half-step measured in microseconds, higher values means less speed!
    
  int speed()
    = gets the current speed
    
  begin()
    = set all coils on, torque will be also on 
    
  end()
    = set all coils off, torque will be also off
    be carefull, the substep() or begin() methods will not take care about the setting of the current to HIGH or LOW
    as a result after calling substep() or begin() the current and the torque stays on 
    which means if you do not care about ... your battery runs dry and your motor will be heated
    
  subStep(long steps)
    turn as many half steps as defined in "long steps", +integers means forward, -integers means backward
    a full step must be multiplied with 2!
    
  setDegree(int degree);
    turns the motor to the specified degrees, 
    having an actual of 0 degrees and setting the value to 720 is exactly 2 full turns forward
    
  double degree()
    = get the actual value of degrees,
    if the value is e.g. 720 (out of all operations beforehand) it is in summary 2 full turns forward
    
  int currentStep()
    = get the actual value of steps done,
    if value is e.g. 2000 (out of all operations beforehand) it is also in summary 2000 steps
*/

//declare an instance of the vma03Stepper class to play with
vma03Stepper myStepper( 3, 2, 
                        9, 8, 
                        200, 
                        5000, 
                        1000);

// some variables used in loop()
int x = 0;
String sBuffer = "";



void setup()
{
  // start serial communication to see whats happening beside your motor
  Serial.begin(9600);
  Serial.println("how many steps/degrees to turn?");
}

void loop(){
  // read your command from serial and store it to sBuffer
  while (Serial.available() > 0) 
  {
    // read the incoming byte:
    sBuffer = sBuffer + char(Serial.read());
  }
  // if there is input turn the motor and do some output on the serial terminal
  if ( sBuffer != "")
  {
    x = sBuffer.toInt();
    myStepper.setDegree(x);
    sBuffer = "";
    myStepper.end();
    someSerialOutput();
  }

}

// do some output on the serial terminal, ensure to have the baud rate set to 9600 as called in Serial.begin(9600)
void someSerialOutput()
{
    Serial.print("Current Step: ");
    Serial.println(myStepper.currentStep());
    Serial.print("Degree: ");
    int y = myStepper.degree();
    Serial.println(y);
    Serial.println("---------------------------------------");
}



