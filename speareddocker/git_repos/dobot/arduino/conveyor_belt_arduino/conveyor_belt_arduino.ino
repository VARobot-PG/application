#include <vma03Stepper.h>
#include <TimerOne.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <arduino_msgs/SetConveyorbeltSpeed.h>
#include <arduino_msgs/get_velocity.h>

#define PUB_FREQ 60

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
const byte IRS_pin = 12; //ir switch pin

//time data to reach publishing frequency
unsigned long time_delta = 0;
unsigned long delay_time = 0;

// initialize the stepper library on pins:
//3,2,9,8
vma03Stepper stepper(3,2,9,8, 200, 1000, 100);
bool enabled = 0;
int8_t direction = 1;

//ros node handle and publisher
ros::NodeHandle nh;
std_msgs::Bool irs_blocked;
ros::Publisher irs("GetInfraredSensorBlocked", &irs_blocked); 
arduino_msgs::get_velocity vel_msg;
ros::Publisher vel("GetConveyorbeltSpeed", &vel_msg);

//conveyorbelt callback
void set_conveyorbelt_speed(const arduino_msgs::SetConveyorbeltSpeed::Request& req, 
                              arduino_msgs::SetConveyorbeltSpeed::Response& res)
{
  vel_msg.velocity = req.speed;
  //speed in %
  float speed = min(100.0F,max(req.speed,-100.0F)) / 100.0F;
  
  if(speed == 0)
    enabled = 0;
  else
  {
    enabled = 1;
    direction = speed > 0;
    Timer1.setPeriod(1500-740*abs(speed));
  }

  res.result = 1;
}

//ros service server for offering setConveyorbeltSpeed service
ros::ServiceServer<arduino_msgs::SetConveyorbeltSpeed::Request, arduino_msgs::SetConveyorbeltSpeed::Response> 
          server("SetConveyorbeltSpeed", &set_conveyorbelt_speed);

void do_one_step()
{
  if(enabled)
    stepper.step_without_waiting(direction);
}

void setup() 
{
  stepper.begin();

  vel_msg.velocity = 0;

  //initialize ros
  //advetise ir switch publisher and stepper service
  nh.initNode();
  nh.advertise(irs);
  nh.advertise(vel);
  nh.advertiseService(server);

  //initialize stepper timer interrupt
  Timer1.initialize(1500);
  Timer1.attachInterrupt(do_one_step);

  //set ir switch pin to pullup mode
  pinMode(IRS_pin, INPUT_PULLUP);

  //compute delay time
  delay_time = 1000000/PUB_FREQ;
}

void loop() 
{
  //read ir switch data
  irs_blocked.data = !digitalRead(IRS_pin);
  
  //maintain publishing frequency
  if(micros() - time_delta > delay_time)
  {
    time_delta = micros();
    irs.publish(&irs_blocked);
    vel.publish(&vel_msg);  
    nh.spinOnce();
  }
}
