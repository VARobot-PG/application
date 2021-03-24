/*
 * Created by philipf
 * Used to test the behavior of the dobot inside the simuilation.
 * The node commands the dobot to build a tower of all objects which are present in the gazebo simulation.
 * */
#include <tuple>
#include <queue>
#include <ros/ros.h>
#include <dobot_msgs/SetPTPCmd.h>
#include <dobot_msgs/SetEndEffectorSuctionCup.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool isIdle = false;
bool isA = false;
ros::Publisher blockTeleporter;

ros::ServiceClient client;
ros::ServiceClient clientSuction;

tf2::Transform armPose = tf2::Transform(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));

tf2::Vector3 stackPosition = tf2::Vector3(0,200,-10);
size_t stackSize = 0;
tf2::Vector3 blockDimensions = tf2::Vector3(38,38,38);

bool listIsSet = false;

std::queue<std::tuple<tf2::Vector3,double>> objectsToGrasp;

void idleCB(const std_msgs::Bool::ConstPtr& msg)
{
	isIdle = msg->data;
}

void poseCB(const geometry_msgs::Pose::ConstPtr& msg)
{
	armPose = tf2::Transform(tf2::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w),
			tf2::Vector3(msg->position.x,msg->position.y,msg->position.z));
}
/*Called by gazebo and contains all models and their state inside the simulator*/
void modelStatesCB(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	if(listIsSet) return;

	size_t index = 0;
	while(index < msg->name.size())
	{
		if(msg->name.at(index).find("block_small") != std::string::npos)
		{
			tf2::Vector3 position(msg->pose.at(index).position.x,msg->pose.at(index).position.y,msg->pose.at(index).position.z);
			tf2::Quaternion rotationQ(msg->pose.at(index).orientation.x,msg->pose.at(index).orientation.y,msg->pose.at(index).orientation.z,msg->pose.at(index).orientation.w);
			double r,p,y;
			tf2::Matrix3x3(rotationQ).getRPY(r,p,y);
			std::cout << msg->name.at(index) << ":" << std::endl;
			printf("Rotation: [%1.2f,%1.2f,%1.2f]\n",r,p,y);
			printf("Position: [%1.2f,%1.2f,%1.2f]\n\n",position.x(),position.y(),position.z());
			objectsToGrasp.push(std::make_tuple(position*1000.0,y));
		}
		index++;
	}
	std::cout << "Found " << objectsToGrasp.size() << " blocks for grasping!\n";
	if(objectsToGrasp.size() > 0) listIsSet = true;
}
/*Teleports a given model inside the simulator*/
void placeBlock(std::string name="block_small")
{
	gazebo_msgs::ModelState m;
	m.model_name = name;
	m.pose.position.x = 0.26;
	m.pose.position.y = 0.075;
	m.pose.position.z = 0.02;
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 0.0;
	m.twist.angular.x = 0.0;
	m.twist.angular.y = 0.0;
	m.twist.angular.z = 0.0;
	m.twist.linear.x = 0.0;
	m.twist.linear.y = 0.0;
	m.twist.linear.z = 0.0;

	blockTeleporter.publish(m);
}
/*Commands the robot to move to a given location with the given mode*/
void moveRobot(tf2::Vector3 goal, double rotation, uint8_t mode=0)
{
	printf("Moving to: [%f,%f,%f]\n",goal.x(),goal.y(),goal.z());
	dobot_msgs::SetPTPCmd::Request req;
	req.ptpMode = mode;
	req.x = goal.x();
	req.y = goal.y();
	req.z = goal.z();
	req.r = 180.0 * rotation / M_PI;
	req.isQueued = true;
	dobot_msgs::SetPTPCmd::Response res;
	client.call(req,res);
}
/* Enables the suctioncup*/
void enableSuction(bool enabled)
{
	dobot_msgs::SetEndEffectorSuctionCup::Request req;
	req.enableCtrl = true;
	req.isQueued = true;
	req.suck = enabled;
	dobot_msgs::SetEndEffectorSuctionCup::Response res;
	clientSuction.call(req,res);
}
/*Busy waiting until the robot is idle*/
void waitUntilRobotIdle(ros::Rate &r)
{
	do{
		ros::spinOnce();
		r.sleep();
	}while(!isIdle && ros::ok());
}
/*Moves the robot to a given coordinate. Blocks the call until a timeout of 2 seconds is reached, or the robot is idle*/
bool modeRobotBlocking(tf2::Vector3 goal, double rotation, ros::Rate r, uint8_t mode=0)
{
	ros::spinOnce();
	double d = armPose.getOrigin().distance(goal);

	if(d > 2.0) moveRobot(goal,rotation,mode);
	else return true;

	double timeout = 10.0;
	ros::spinOnce();
	while((!isIdle || (d >= 2.0)) && ros::ok())
	{
		r.sleep();
		ros::spinOnce();
		d = armPose.getOrigin().distance(goal);
		if(isIdle)
		{
			timeout -= r.expectedCycleTime().toSec();
		}
		if(timeout < 0.0)
		{
			std::cout << "Position timeout!\n";
			printf("Distance: %f Goal: [%f,%f,%f], Current: [%f,%f,%f]\n",
					d,goal.x(),goal.y(),goal.z(),armPose.getOrigin().x(),armPose.getOrigin().y(),armPose.getOrigin().z());
			return false;
		}
	}
	return true;
}
/*Grasps the block and adds it to the tower of blocks*/
bool addNewBlockToStack(tf2::Vector3 blockPosition, double blockRotation, ros::Rate r)
{
	std::cout << "Collecting!\n";
	if(!modeRobotBlocking(blockPosition,blockRotation,r) || !ros::ok()) return false;
	ros::Duration(0.1).sleep();
	std::cout << "Sucking!\n";
	enableSuction(true);
	ros::Duration(1.0).sleep();

	std::cout << "Placing!\n";
	if(!modeRobotBlocking(stackPosition + (stackSize * blockDimensions*tf2::Vector3(0,0,1)),0.0,r) || !ros::ok()) return false;
	stackSize++;
	ros::Duration(1.0).sleep();
	std::cout << "Placing!\n";
	enableSuction(false);
	ros::Duration(1.0).sleep();
	return true;
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "dobotTest");

    ros::NodeHandle nh;
    ros::Rate r(5);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    blockTeleporter = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);
    client = nh.serviceClient<dobot_msgs::SetPTPCmd>("/Dobot_Loader/SetPtpCmd");
    clientSuction = nh.serviceClient<dobot_msgs::SetEndEffectorSuctionCup>("/Dobot_Loader/SetEndEffectorSuctionCup");
    ros::Subscriber idleSub = nh.subscribe("/Dobot_Loader/idle",2,idleCB);
    ros::Subscriber modelStateSub = nh.subscribe("/gazebo/model_states",1,modelStatesCB);
    ros::Subscriber roborPoseCB = nh.subscribe("/Dobot_Loader/arm/Pose",1,poseCB);

    moveRobot(tf2::Vector3(200,0,50),0,1);
    enableSuction(false);

    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ros::spinOnce();

    waitUntilRobotIdle(r);
    ros::Duration(1.0).sleep();

    bool placedBuilding = false;

    while (ros::ok())
    {
    	while(objectsToGrasp.size() > 0 && ros::ok())
    	{
    		std::cout << "Taking object id " << objectsToGrasp.size() << std::endl;
    		auto object = objectsToGrasp.front();

    		geometry_msgs::TransformStamped transformStamped;
			try{
			  transformStamped = tfBuffer.lookupTransform("Dobot_Loader/world", "Dobot_Loader/Dobot_Loader",ros::Time(0));
			}
			catch (tf2::TransformException &ex) {
			  ROS_WARN("%s",ex.what());
			  ros::Duration(1.0).sleep();
			  continue;
			}
			tf2::Stamped<tf2::Transform> worldToRobot;
			tf2::fromMsg(transformStamped,worldToRobot);
    		addNewBlockToStack(std::get<0>(object)-tf2::Vector3(0,0,6 + blockDimensions.z()),std::get<1>(object),r);
    		objectsToGrasp.pop();
    		placedBuilding = true;
    	}
    	ros::spinOnce();
    	r.sleep();

    	if(placedBuilding)//If the building is done, destroy it!
    	{
    		modeRobotBlocking(tf2::Vector3(-75,200,-30),0.0,r);
    		ros::Duration(1.0).sleep();
    		ros::spinOnce();
    		modeRobotBlocking(tf2::Vector3(75,200,-30),0.0,r,2);
    		ros::Duration(2.0).sleep();
    		ros::spinOnce();

    		placedBuilding = false;
    		listIsSet = false;
    		stackSize = 0;
    	}
    }
    return 0;
}
