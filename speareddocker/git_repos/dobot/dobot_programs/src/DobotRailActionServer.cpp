#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdlib>
#include <dobot_msgs/PickupActionResult.h>
#include "../include/PickupAction.h"
#include "../include/PlaceInStorageAction.h"
#include "../include/ScanColorAction.h"
#include "../include/Action.h"
#include <dobot/dobot.h>
#include <arduino/arduino.h>

const std::string ROBOT_NAME = dobot::dobot_names::DOBOT_RAIL;

dobot::Dobot dobot1(ROBOT_NAME);
std::shared_ptr<ros::NodeHandle> nh_ptr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pickup");
    ros::NodeHandle nh;
    nh_ptr = std::make_shared<ros::NodeHandle>(nh);

    arduino::SensorNode sensor_node(nh_ptr);
    arduino::RFIDNode rfid_node(nh_ptr);

    dobot1.initArm(nh_ptr,ROBOT_NAME);
    dobot1.enableSuctionCup(false);
    dobot1.moveArmToPositionWithL(dobot::env::R_WAIT_POSITION, 0, 0);

    ROS_INFO("DobotRailActionServer: Advertising actions!");
    PickupAction pickup("pickup",nh_ptr, dobot1);
    PlaceInStorageAction place("place",nh_ptr, dobot1);
    ScanColorAction scan("scanColor",nh_ptr, dobot1, sensor_node);
    ScanRFIDAction rfid("scanRFID", nh_ptr, dobot1,rfid_node);
    TruckBuildAction truck_build("truckBuild", nh_ptr, dobot1);
    TruckStoragePlaceAction truck_storage_place("truckStoragePlace", nh_ptr, dobot1);
    TruckStorageTakeAction  truck_storage_take("truckStorageTake", nh_ptr, dobot1);
    ROS_INFO("RightDobotActionServer: Starting endless loop.");

    while(ros::ok())
        ros::spinOnce();
    return 0;
}
