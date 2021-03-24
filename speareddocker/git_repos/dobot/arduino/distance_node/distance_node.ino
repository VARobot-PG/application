#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <ros.h>
#include <Adafruit_TCS34725.h>
#include <arduino_msgs/rgb_color.h>
#include <arduino_msgs/distance.h>

#define PUB_FREQ 60
//custom I²C addresses of distance sonsors
#define MULTIPLEXER_ADDRESS 0x70

//multiplexer ports of sensors
#define DISTANCE_SENSOR_RIGHT_PORT 7
#define DISTANCE_SENSOR_LEFT_PORT 1
#define COLOR_SENSOR_PORT 2

unsigned long time_delta = 0;
unsigned long delay_time = 0;

//ros node handle and msgs
ros::NodeHandle nh;
arduino_msgs::rgb_color rgb_color;
arduino_msgs::distance dist_sensor_left;
arduino_msgs::distance dist_sensor_right;

//ros publisher
ros::Publisher color("GetRGBColor", &rgb_color);
ros::Publisher distance_right("GetDistanceSensorRight", &dist_sensor_right);
ros::Publisher distance_left("GetDistanceSensorLeft", &dist_sensor_left);

//right distance sensor (I²C address: 0x50)
Adafruit_VL6180X vl_r = Adafruit_VL6180X();
Adafruit_VL6180X vl_l = Adafruit_VL6180X();

//rgb sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
uint16_t r,g,b;

//selects the port on the multiplexer (0-7)
void multiplexer_port_select(uint8_t port)
{
  if(port > 7)
    return;

  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << port);
  Wire.endTransmission();
}

void setup() 
{
  delay(1000);
  Wire.begin();
  
  multiplexer_port_select(DISTANCE_SENSOR_RIGHT_PORT);
  vl_r.begin();

  multiplexer_port_select(DISTANCE_SENSOR_LEFT_PORT);
  vl_l.begin();

  multiplexer_port_select(COLOR_SENSOR_PORT);
  tcs.begin();

  delay_time = 1000000/PUB_FREQ;

  //initialize ros and advertise publisher
  nh.initNode();
  nh.advertise(color);
  nh.advertise(distance_right);
  nh.advertise(distance_left);
}

void loop() {
  //rgb sensor data
  tcs.getRawData(&r, &g, &b, &rgb_color.c);
  rgb_color.r = (uint8_t)r;
  rgb_color.g = (uint8_t)g;
  rgb_color.b = (uint8_t)b;
  
  //right distance sensor data
  multiplexer_port_select(DISTANCE_SENSOR_RIGHT_PORT);
  if(vl_r.sample_available())
    dist_sensor_right.dist = vl_r.readRangeNonBlocking();
  dist_sensor_right.status = vl_r.readRangeStatus();

  //left distance sensor data
  multiplexer_port_select(DISTANCE_SENSOR_LEFT_PORT);
  if(vl_l.sample_available())
    dist_sensor_left.dist = vl_l.readRangeNonBlocking();
  dist_sensor_left.status = vl_l.readRangeStatus();
  
  //maintain publishing frequency
  if(micros() - time_delta > delay_time)
  {
    time_delta = micros();
    color.publish(&rgb_color);
    distance_right.publish(&dist_sensor_right);
    distance_left.publish(&dist_sensor_left);
    nh.spinOnce();
  }

  multiplexer_port_select(COLOR_SENSOR_PORT);
}
