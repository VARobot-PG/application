#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dobot_msgs/ScanColorAction.h>
#include <dobot_msgs/ScanColorActionFeedback.h>
#include <dobot_msgs/ScanColorActionResult.h>
#include <boost/bind.hpp>
#include <arduino_msgs/rgb_color.h>
#include <arduino/arduino.h>
#include <dobot/dobot.h>

class ScanColorAction
{
protected:

    actionlib::SimpleActionServer<dobot_msgs::ScanColorAction> ps_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    const std::string action_name_;
    std::shared_ptr<ros::NodeHandle> nh;
    dobot::Dobot &dobot1;
    arduino::SensorNode &color_node;

    // create messages that are used to publish feedback/result
    dobot_msgs::ScanColorFeedback feedback_;
    dobot_msgs::ScanColorResult result_;

    double SCAN_COLOR_LINEARRAIL_POS = 0.0;

public:

    ScanColorAction(std::string name, std::shared_ptr<ros::NodeHandle> gNh, dobot::Dobot &d, arduino::SensorNode &a) :
            ps_(*gNh, name, boost::bind(&ScanColorAction::pickupBlockCB, this, _1), false),
            action_name_(name),
			nh(gNh),
			dobot1(d),
			color_node(a)
    {
    	std::cout << "ScanColorAction\n";
    	ps_.registerPreemptCallback(boost::bind(&ScanColorAction::preemptCB,this));
        ps_.start();
    }

    ~ScanColorAction(void)
	{
	}

    void preemptCB()
    {
    	ROS_INFO("%s: Preempted", action_name_.c_str());
    	// set the action state to preempted
    	ps_.setPreempted();
    }

    void pickupBlockCB(const dobot_msgs::ScanColorGoalConstPtr &goal)
    {
        ROS_INFO("Executing, picking up block at position [%f,%f,%f,%f]",
        		dobot::env::SCAN_COLOR_POSITION.x(),dobot::env::SCAN_COLOR_POSITION.y(),
        		dobot::env::SCAN_COLOR_POSITION.z(),SCAN_COLOR_LINEARRAIL_POS);
        // start executing the action
        feedback_.comment.data = "Moving to Pickup";
        feedback_.progress = 0.1;
        ps_.publishFeedback(feedback_);

        /* move arm to pickup position */
        while(!dobot1.isAtPosition(dobot::env::SCAN_COLOR_POSITION) && ros::ok()){ //TODO stop if position can not be reached //TODO Calculate distance to target position and return as progress
        	if(dobot1.isIdle()){
				dobot1.moveArmToPositionWithL(dobot::env::SCAN_COLOR_POSITION, goal->rotation, SCAN_COLOR_LINEARRAIL_POS);
        	}
        	ros::Duration(1).sleep();
        }
        feedback_.comment.data = "Scanning color";
        feedback_.progress = 0.9;
        ps_.publishFeedback(feedback_);
        //Read out rgb sensor
        ros::Duration(0.5).sleep(); //Lets give some time to the sensor to adapt

        if(!color_node.read_rgb_sensor(result_.r, result_.g, result_.b))
        {
        	feedback_.comment.data = "Error! No RGB value received!";
        	feedback_.progress = 1.0;
        	ps_.publishFeedback(feedback_);
        	ROS_ERROR("Unable to connect with RGB sensor!");
        	ps_.setAborted();
        }else{
			result_.success = true;
            std::cout << "Result: r: " << (int)result_.r << " g: " << (int)result_.g << " b: "<< (int)result_.b << std::endl;
			feedback_.comment.data = "Done";
			feedback_.progress = 1.0;
			ps_.publishFeedback(feedback_);
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			ps_.setSucceeded(result_); // set the action state to succeeded
        }
    }
};
