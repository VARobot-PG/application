#include "vma03Stepper.h"

vma03Stepper::vma03Stepper( byte pwmaPin, byte diraPin, 
                            byte pwmbPin, byte dirbPin, 
                            int stepsPerRevolution, 
                            int speed, 
                            int speedOffset)
{
  _pwmaPin = pwmaPin;
  _diraPin = diraPin;
  _pwmbPin = pwmbPin;
  _dirbPin = dirbPin;
  _stepsPerRevolution = stepsPerRevolution;
  _speed = speed;
  _speedOffset = speedOffset;
  _degree = 0;
	
	STEPS = 2 * _stepsPerRevolution;
  
  // ensure to have the delay between the steps at least at speedOffest
  if (_speed < _speedOffset)
    {
      _speed = _speedOffset;
    }
  pinMode(_pwmaPin, OUTPUT);
  pinMode(_diraPin, OUTPUT);
  pinMode(_pwmbPin, OUTPUT);
  pinMode(_dirbPin, OUTPUT);

}

int vma03Stepper::speed()
{
  return _speed;
}

void vma03Stepper::setSpeed(int newSpeed)
{
  if (newSpeed < _speedOffset)
    {
      _speed = _speedOffset;
    }
  else
    {
      _speed = newSpeed;
    }
}

void vma03Stepper::begin(){
  digitalWrite(_pwmaPin, HIGH);
  digitalWrite(_diraPin, HIGH);
  digitalWrite(_pwmbPin, HIGH);
  digitalWrite(_dirbPin, HIGH);
}

void vma03Stepper::end()
{
  digitalWrite(_pwmaPin, LOW);
  digitalWrite(_diraPin, LOW);
  digitalWrite(_pwmbPin, LOW);
  digitalWrite(_dirbPin, LOW);
}

int vma03Stepper::currentStep()
{
  return _currStep;
}

double vma03Stepper::degree()
{
  double r = _currStep * (360.00 / _stepsPerRevolution / 2);
  return r;
}

void vma03Stepper::setDegree(int degree)
{
  double futureStep =  0.00;
  futureStep = degree / 360.00 * _stepsPerRevolution * 2;
  int stepsToGo = futureStep - _currStep;
  subStep(stepsToGo);
}


// This method is called in order to make the stepper motor make a number of sub steps (depending on your wiring).
// Variable steps is for number of steps (forwards = positive, backwards = negative)
// stepDelay is for waiting time between steps in microseconds => bigger means lower speed, smaller means higher speed

void vma03Stepper::do_step(int current_step)
{
	switch(current_step)
        {
        case 0: 
          // Starting position (if repeated, full step (4))
          // in this case, both our power are high.
          // Therefore both coils are activated, with their standard polarities for their magnetic fields.
          digitalWrite(_pwmaPin,HIGH);
          digitalWrite(_pwmbPin,HIGH);
          digitalWrite(_diraPin,HIGH);
          digitalWrite(_dirbPin,HIGH);
          break;

        case 1:
          //Half step (½)
          //In this case, only out b-coil is active, still with it's stand polarity.
          digitalWrite(_pwmaPin,HIGH);
          digitalWrite(_pwmbPin,LOW);
          digitalWrite(_diraPin,HIGH);
          digitalWrite(_dirbPin,LOW);
          break;
	
        case 2:
          //Full step (1)
          // In this case, the b-coil is activated as in previous cases.
          // But the a-coil now has it's direction turned on. So now it's active, but with the reversered polarity.
          // By continuing this pattern (for reference: http://www.8051projects.net/stepper-motor-interfacing/full-step.gif) , you'll get the axis to turn.
          digitalWrite(_pwmaPin,HIGH);
          digitalWrite(_pwmbPin,HIGH);
          digitalWrite(_diraPin,HIGH);
          digitalWrite(_dirbPin,LOW);
          break;

        case 3:
          // Half step (1½)
          digitalWrite(_pwmaPin,LOW);
          digitalWrite(_pwmbPin,HIGH);
          digitalWrite(_diraPin,LOW);
          digitalWrite(_dirbPin,LOW);
          break;

        case 4:
          // Full step (2)
          digitalWrite(_pwmaPin,HIGH);
          digitalWrite(_pwmbPin,HIGH);
          digitalWrite(_diraPin,LOW);
          digitalWrite(_dirbPin,LOW);
          break;

        case 5:
          // Half step (2½)
          digitalWrite(_pwmaPin,HIGH);
          digitalWrite(_pwmbPin,LOW);
          digitalWrite(_diraPin,LOW);
          digitalWrite(_dirbPin,LOW);
          break;

        case 6:
          // Full step (3)
          digitalWrite(_pwmaPin,HIGH);
          digitalWrite(_pwmbPin,HIGH);
          digitalWrite(_diraPin,LOW);
          digitalWrite(_dirbPin,HIGH);
          break;

        case 7:
          // Half step (3½)
          digitalWrite(_pwmaPin,LOW);
          digitalWrite(_pwmbPin,HIGH);
          digitalWrite(_diraPin,LOW);
          digitalWrite(_dirbPin,HIGH);
          break;
         }
}

void vma03Stepper::step_without_waiting(byte direction)
{
	if(direction)
		_currStep--;       //increment current halfstep (forward)
  else
		_currStep++;       //decrement current halfstep (backward)

  if(_currStep>STEPS)
		_currStep= _currStep-STEPS;         //position >360deg is reached => set position one turn back
  else if(_currStep<0)
		_currStep= _currStep+STEPS;             //position <0deg   is reached => set position one turn forward
	
	_sub = _currStep%8;

	do_step(_sub);
}

void vma03Stepper::subStep(long steps){

  // The function will run for the amount of times called in the method.
  // This is accomplished by a while loop, where it will subtract 1 from the amount after every run (forwards).
  // In case of backward rotation it will add 1 to the negative number of steps until 0 is reached.

		byte direction = 1;

		if(steps < 0 )
			direction = 0;

		steps = abs(steps);

    while(steps!=0)
    {
        if(direction)
					_currStep--;       //increment current halfstep (forward)
        else
					_currStep++;       //decrement current halfstep (backward)

        if(_currStep>STEPS)
					_currStep= _currStep-STEPS;         //position >360deg is reached => set position one turn back
				else if(_currStep<0)
					_currStep= _currStep+STEPS;             //position <0deg   is reached => set position one turn forward

				
        _sub = _currStep%8;           //determine the next halfstep

        do_step(_sub);

        delayMicroseconds(_speed);        //Waiting time to next halfstep

        steps--;      //decrement of remaining halfsteps of forward rotation
    }
}

