//
// Created by speeters on 08.05.19.
//

#ifndef SRC_ACTION_H
#define SRC_ACTION_H

#include <arduino/arduino.h>
#include <dobot/dobot.h>
#include <tf2/LinearMath/Transform.h>
#include <actionlib/server/simple_action_server.h>
#include <dobot_msgs/ScanRFIDAction.h>
#include <dobot_msgs/TruckBuildAction.h>
#include <dobot_msgs/TruckStoragePlaceAction.h>
#include <dobot_msgs/TruckStorageTakeAction.h>
#include <tf_helper/tfHelper.h>
#include <marker_helper/markerHelper.h>
#include <dobot_msgs/TruckBuildGoal.h>
#include <dobot_msgs/TruckBuildResult.h>


template <class U>  class Action{
protected:
    std::shared_ptr<ros::NodeHandle> node_handle;
    std::string action_name;
    dobot::Dobot &dobot;

    Action(std::string name, std::shared_ptr<ros::NodeHandle> handle, dobot::Dobot &dob) :
            action_name(name), node_handle(handle), dobot(dob){
    }

public:
    virtual void callback(const U &goal) = 0;
};


/**
 * This class represents the Action to scan a block with the rfid sensor
 */
class ScanRFIDAction : public Action<dobot_msgs::ScanRFIDGoalConstPtr>
{
private:
    arduino::RFIDNode& arduino_node;
    actionlib::SimpleActionServer<dobot_msgs::ScanRFIDAction> action_server;
    dobot_msgs::ScanRFIDResult result;

public:
    ScanRFIDAction(std::string name, std::shared_ptr<ros::NodeHandle> handle, dobot::Dobot &dob, arduino::RFIDNode& node);
    void callback(const dobot_msgs::ScanRFIDGoalConstPtr &goal) override;

};


/**
 * This class represents the action to build the lkw.
 */

class TruckBuildAction : public Action<dobot_msgs::TruckBuildGoalConstPtr>
{
protected:
    actionlib::SimpleActionServer<dobot_msgs::TruckBuildAction> action_server;

    dobot_msgs::TruckBuildResult result;
    std::unique_ptr<TfHelper> tfHelper;
    std::unique_ptr<MarkerHelper> markerManager;

public:
    TruckBuildAction(std::string name, std::shared_ptr<ros::NodeHandle> handle, dobot::Dobot &dob);
    void callback(const dobot_msgs::TruckBuildGoalConstPtr &goal);

};

/**
 * Truck Storage Action
 */
class TruckStoragePlaceAction : public Action<dobot_msgs::TruckStoragePlaceGoalConstPtr>
{
protected:
    actionlib::SimpleActionServer<dobot_msgs::TruckStoragePlaceAction> action_server;
    dobot_msgs::TruckStoragePlaceResult result;
    float linear_rail_base = 300;

public:
    TruckStoragePlaceAction(std::string name, std::shared_ptr<ros::NodeHandle> handle, dobot::Dobot &dob);
    void callback(const dobot_msgs::TruckStoragePlaceGoalConstPtr & goal);
};

/**
 * TruckStorageTakeAction
 */

class TruckStorageTakeAction : public Action<dobot_msgs::TruckStorageTakeGoalConstPtr>
{
protected:
    actionlib::SimpleActionServer<dobot_msgs::TruckStorageTakeAction> action_server;
    dobot_msgs::TruckStorageTakeResult result;
    float linear_rail_base = 300;

public:
    TruckStorageTakeAction(std::string name, std::shared_ptr<ros::NodeHandle> handle, dobot::Dobot &dob);
    void callback(const dobot_msgs::TruckStorageTakeGoalConstPtr & goal);
};






#endif //SRC_ACTION_H
