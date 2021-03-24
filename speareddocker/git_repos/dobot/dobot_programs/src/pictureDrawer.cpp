//
// Created by lars on 05.02.19.
//

#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <iostream>
#include <string>
#include <memory>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include "../include/pictureParser.h"
#include <dobot/dobot.h>
#include <arduino/arduino.h>
#include <dobot_msgs/PickupAction.h>
#include <dobot_msgs/ScanColorAction.h>
#include <dobot_msgs/ScanRFIDAction.h>
#include <dobot_msgs/PlaceInStorageAction.h>
#include <detection/detection.h>
#include <detection_msgs/DetectedObject.h>
#include <tf_helper/tfHelper.h>
#include <cmath>

//width and height of the dobot picture
const size_t PIC_WIDTH = 6;
const size_t PIC_HEIGHT = 3;
const std::string ROBOT_NAME = dobot::dobot_names::DOBOT_RAIL;
//top left corner of picture
const tf2::Vector3 PIC_BASE_POSITION = tf2::Vector3(230.0,-200.0,-110.0); //-123
const tf2::Vector3 GARBAGE_POSITION = tf2::Vector3(134.0,-650.0,-110.0); //-109

const double PROBABILITY_THRESHOLD = 0.9;
std::unique_ptr<TfHelper> tfHelper;
Storage garbage(GARBAGE_POSITION, 3, 3, "Garbage");
bool newBlock = true;


tf2Scalar to_degrees(tf2Scalar radians) { return radians * 180 / M_PI; }

/**
 * callback for pick up done info
 * @param state state of the action
 * @param result result of the action
 */
void pickup_done(const actionlib::SimpleClientGoalState &state,
                const dobot_msgs::PickupResultConstPtr &result)
{
    ROS_INFO("Dobot %s pickup action!", result->success?"succeded":"failed");
}


/**
 * action feedback callback
 * @param feedback feedback of pickup action
 */
void pickup_feedback(const dobot_msgs::PickupFeedbackConstPtr &feedback)
{
    ROS_INFO("Progress: %f Comment: %s",feedback->progress ,feedback->comment.data.c_str());
}


/**
 * pick up active callback
 */
void pickup_active()
{
    ROS_INFO("Started pickup action");
}

/**
 * picks up the block
 * @return true iff action was successful
 */
bool pickup_block(ros::NodeHandle &nh, dobot::Dobot& arm, detection::RailGraspObjectGetter& graspObjectGetter, ros::Time latest)
{
    tf2::Transform transform;
    if(!tfHelper->getTransform(ROBOT_NAME + "_tool",graspObjectGetter.getFrameID(),ros::Time(0),transform)) return false;

    std::cout << "Creating server!\n";

    actionlib::SimpleActionClient<dobot_msgs::ScanRFIDAction> clientTest("scanRFID");
    std::cout << clientTest.isServerConnected() << std::endl;
    std::cout << "Waiting server!\n";
    clientTest.waitForServer();
    std::cout << "Done server!\n";

    return false;

    ros::Duration(1.0).sleep();
    actionlib::SimpleActionClient<dobot_msgs::PickupAction> client(nh,"pickup",true);

    ros::Duration(1.0).sleep();
    std::cout << client.isServerConnected() << std::endl;
    std::cout << "Waiting server!\n";
    client.waitForServer();
    std::cout << "Done server!\n";

    //get the right most object
    detection_msgs::DetectedObject obj = graspObjectGetter.getRightMostObject(ros::Time(0), PROBABILITY_THRESHOLD);
    tf2::Vector3 graspPoint, point;
    
    while(obj.probability < 0)
    {
        ros::spinOnce();
        obj = graspObjectGetter.getRightMostObject(ros::Time(0), PROBABILITY_THRESHOLD);
    }

    //get rotation
    tf2Scalar roll=0, pitch=0, yaw=0;
    tf2::Quaternion quat;
    tf2::fromMsg(obj.rotation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    ROS_INFO("frame id: %s", graspObjectGetter.getFrameID().c_str());
    ROS_INFO("point: (%f,%f,%f)", obj.graspPoint.x, obj.graspPoint.y, obj.graspPoint.z);
    ROS_INFO("probability: %f", obj.probability);
    ROS_INFO("rotation: %f degrees", to_degrees(yaw));

    //get tf2::Vector3 from geometry_msgs::Vector3
    tf2::fromMsg(obj.graspPoint, point);
    //convert to local dobot coordinates with tool offset
    double railPos=0;
    arm.convertToDobotPointWithL(point, graspPoint, railPos, graspObjectGetter.getFrameID());
    ROS_INFO("converted point: (%f,%f,%f)", graspPoint.x(), graspPoint.y(), graspPoint.z());

    dobot_msgs::PickupGoal goal;
    goal.position.x = graspPoint.x();
    goal.position.y = graspPoint.y()+5; //TODO: offset not always working due to object position being imprecise
    goal.position.z = graspPoint.z()-5;
    goal.rotation = fmod(-90+to_degrees(yaw), 90);
    client.sendGoal(goal, &pickup_done, &pickup_active, &pickup_feedback);

    return client.waitForResult(ros::Duration(20.0));
}


/**
 * scans the color
 * @param r red value read
 * @param g green value read
 * @param b blue value read
 * @return true iff reading color was successful
 */
bool scan_color(uint8_t &r, uint8_t &g, uint8_t &b)
{
    actionlib::SimpleActionClient<dobot_msgs::ScanColorAction> client("scanColor");

    ROS_INFO("Waiting for scan color server");
    client.waitForServer();

    ROS_INFO("Executing Scan Color Action!");
    dobot_msgs::ScanColorGoal goal;
    goal.rotation = 0;
    client.sendGoal(goal);

    bool success = client.waitForResult(ros::Duration(10.0));

    auto result = client.getResult();

    r = result->r;
    g = result->g;
    b = result->b;

    ROS_INFO("Read r=%u, g=%u, b=%u", r, g, b);

    return success;
}


/**
 * Stores the block at the specified location
 * @param location the location to store the block at
 * @param rotation the rotation of the block to correct to 0 degrees
 * @return true iff action has been successful
 */
bool storeBlock(tf2::Vector3 location, double rotation)
{
    actionlib::SimpleActionClient<dobot_msgs::PlaceInStorageAction> client("place");

    ROS_INFO("Waiting for action PlaceInStorage.");
    client.waitForServer(); //will wait for infinite time
    ROS_INFO("PlaceInStorage Action server started, sending goal.");

    dobot_msgs::PlaceInStorageGoal goal;
    goal.position.x = location.x();
    goal.position.y = location.y();
    goal.position.z = location.z();
    goal.rotation = rotation;
    client.sendGoal(goal);

    //wait for the action to return
    bool timeout = !client.waitForResult(ros::Duration(20.0));
    if(timeout)
    {
        ROS_ERROR("PlaceInStorage Action timeout!");
        return false;
    }

    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("PlaceInStorage Action finished: %s",state.toString().c_str());
    return true;
}


std::string color_of(PictureColor c)
{
    switch(c)
    {
        case PictureColor::BLUE:
            return "blue";
        case PictureColor::RED:
            return "red";
        case PictureColor::GREEN:
            return "green";
        case PictureColor::YELLOW:
            return "yellow";
        case PictureColor::UNKNOWN:
            return "unknown";
    }
}

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        ROS_ERROR("You need to specify exactly one file to load!");
        return -1;
    }

    //setup picture to draw
    std::string file = std::string(argv[1]);
    PictureParser parser(file);
    Picture pic = parser.generate_dobot_picture(PIC_WIDTH, PIC_HEIGHT);

    ROS_INFO("Drawing following picture: ");
    pic.ostream_show(std::cout);

    ros::init(argc, argv, "pictureDrawer");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh);
    arduino::ConveyorNode if_node(nh_ptr);
    detection::RailGraspObjectGetter graspObjectGetter(nh_ptr);
    dobot::Dobot arm(ROBOT_NAME);
    arm.initArm(nh_ptr, ROBOT_NAME);
    tfHelper = std::make_unique<TfHelper>();

    bool success = false;
    ros::Time time = ros::Time::now();

    ROS_INFO("Starting loop");
    while(ros::ok())
    {
        ROS_INFO("Waiting for block!");
        do{
            ros::spinOnce();
            ros::Duration(0.25).sleep();
        }while(!if_node.is_if_sensor_blocked() && ros::ok());

        //update timestamp for new block to delete old positions of this block
        if(if_node.is_if_sensor_blocked() && newBlock)
        {
            time = ros::Time::now();
            newBlock = false;
        }

        if(if_node.is_if_sensor_blocked() && !graspObjectGetter.isEmpty())
        {
            ROS_INFO("Block found!");
            success = pickup_block(nh,arm, graspObjectGetter, time);

            if(!success)
            {
                ROS_ERROR("Could not communicate with action server or block not found! Trying again...");
                break;
            }

            uint8_t r=0,g=0,b=0;
            success = scan_color(r, g, b);

            if(!success)
            {
                ROS_ERROR("Could not communicate with action server. Trying again...");
                break;
            }

            PictureColor color = pic::convert_rgb_to_pic_color(r, g, b);
            tf2::Vector3 location;
            ROS_INFO("Found color: %s", color_of(color).c_str());

            if(pic.is_color_needed(color))
            {
                size_t row = 0, col = 0;
                location = pic.get_next_set_position(PIC_BASE_POSITION, color, dobot::env::R_BLOCK_SIZE, row, col);
                pic.set_color_set(col, row);
                ROS_INFO("Choosing storage: %s. Block location: [%f, %f, %f]",
                         "Flag",location.x(),location.y(),location.z());
            }
            else{
                location = garbage.getItemPosition(garbage.itemCount++, dobot::env::R_BLOCK_SIZE);
                ROS_INFO("Choosing storage: %s. Block id: %i. Block location: [%f, %f, %f]",
                         garbage.name.c_str(),garbage.itemCount-1,location.x(),location.y(),location.z());
            }

            double rotation = 0.0;
            storeBlock(location, rotation);
            newBlock = true;
        }

        ros::spinOnce();
        //idle position
        if(!if_node.is_if_sensor_blocked())
        {
            //TODO: fix going back to waiting position (for some unknown reason action server times out)
            ROS_INFO("Going to waiting position");
            storeBlock(dobot::env::R_PICTURE_WAIT, 0.0);
        }
    }

    return 0;
}

