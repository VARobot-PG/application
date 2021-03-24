/**
 * dobotCalibration.cpp
 *
 *  Created on: June 06, 2019
 *      Author: philipf
 *
 *
 */
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/poses_from_matches.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <geometry_msgs/Pose.h>

#include <detection_msgs/SetFrame.h>
#include <detection_msgs/SetString.h>
#include <detection_msgs/SetPointcloud.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>

#include <dobot/dobot.h>
#include "../include/cloud.h"

const float UPDATE_RATE = 30.0;
//Definition of speeds for moving the frame.
const double JOY_SPEED_NORMAL = 0.005;
const double JOY_SPEED_SLOW = JOY_SPEED_NORMAL * 0.025;
const double JOY_SPEED_FAST = JOY_SPEED_NORMAL * 5;
//Position of cursor
tf2::Transform currentTransform = tf2::Transform(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));
tf2::Transform solvedTransform = tf2::Transform(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));
tf2::Vector3 armPosition;
tf2::Vector3 armPositionDobotFrame;
double railPositionDobotFrame = 0.0;
double commandedRailPosDobotFrame = 1.2345;

std::shared_ptr<ros::NodeHandle> nh;
tf2_ros::Buffer tfBuffer;
pcl::visualization::PCLVisualizer::Ptr viewer;
pcl::PointCloud<cloud::PointType>::Ptr sampledCloud;
ros::ServiceClient saveFrames;
ros::ServiceClient setFrame;
ros::ServiceClient registerPointcloudSubscriber;

std::string frameName = "Dobot_Loader";
std::string cameraName = "kinect2_jetson";
std::string dobotFrame = "Dobot_Loader";

bool recording = false;
bool updatedCloud = false;
bool init = true;
bool selectCloudPoint = true;
bool allowDobotMove = false;
bool clickMoveMode = false;

std::vector<tf2::Vector3> worldPoints;
std::vector<tf2::Vector3> dobotPoints;
std::vector<tf2::Vector3> transPoints;
std::vector<std::string> drawnPoints;

sensor_msgs::JoyConstPtr joyMessage;
bool joyMessageValid = false;

//Buttons should be only processed once. This array sets a button to true, if it already has been processed.
bool buttonsProcessed[12] = {false,false,false,false,false,false,false,false,false,false,false,false};

dobot::Dobot arm(dobot::dobot_names::DOBOT_LOADER);

/**
 * Joystick callback.
 * @param joy
 */
void joyCallback (const sensor_msgs::JoyConstPtr &joy)
{
	joyMessage = joy;
	joyMessageValid = true;
}


void armCallback (const geometry_msgs::PoseConstPtr &msg)
{
	armPositionDobotFrame = tf2::Vector3(msg->position.x,msg->position.y,msg->position.z);
}

void railCallback (const std_msgs::Float64ConstPtr &msg)
{
	railPositionDobotFrame = -msg->data;
	if(commandedRailPosDobotFrame == 1.2345)//Set variable on initialization
	{
		commandedRailPosDobotFrame = railPositionDobotFrame;
	}
}

void removePoints()
{
	for(auto s : drawnPoints)
	{
		viewer->removeShape(s,0);
	}
}
void drawPoint(std::string id, tf2::Vector3 pos, double r, double g, double b)
{
	pcl::PointXYZ p;
	p.x = pos.x();
	p.y = pos.y();
	p.z = pos.z();
	viewer->addSphere(p,0.005,r,g,b,id,0);
}
void drawPoints()
{
	removePoints();
	for(size_t i = 0; i < dobotPoints.size(); i++)
	{
		std::string id = std::to_string(i) + "_dobot";
		drawnPoints.push_back(id);
		drawPoint(id,dobotPoints.at(i),0,0,1);
	}

	for(size_t i = 0; i < worldPoints.size(); i++)
	{
		std::string id = std::to_string(i) + "_world";
		drawnPoints.push_back(id);
		drawPoint(id,worldPoints.at(i),0,1,0);
	}

	for(size_t i = 0; i < transPoints.size(); i++)
	{
		std::string id = std::to_string(i) + "_trans";
		drawnPoints.push_back(id);
		drawPoint(id,transPoints.at(i),1,1,0);
	}
}

void minimizeWithEigen()
{
	Eigen::MatrixXd start,end;
	start.conservativeResize(3,dobotPoints.size());
	end.conservativeResize(3,dobotPoints.size());
	for(size_t i = 0; i < dobotPoints.size(); i++)
	{
		end.col(i) = Eigen::Vector3d(worldPoints.at(i).x(),worldPoints.at(i).y(),worldPoints.at(i).z());
		start.col(i) = Eigen::Vector3d(dobotPoints.at(i).x(),dobotPoints.at(i).y(),dobotPoints.at(i).z());
	}
	Eigen::Matrix<double,4,4> solution = Eigen::umeyama(start,end);
	std::cout << solution.matrix() << std::endl;
	Eigen::Affine3d t;
	t = solution;
//	tf2::convert(t,solvedTransform);
}

/**
 * Redraws all coordinate frames and text!
 */
void drawOrigin()
{
	Eigen::Affine3d t;
	geometry_msgs::Transform gT;
	tf2::convert(currentTransform,gT);
	t = tf2::transformToEigen(gT);
	Eigen::Affine3f affine = t.cast<float>();
	viewer->removeCoordinateSystem("origin",0);
	viewer->addCoordinateSystem (0.1, affine,"origin", 0);

	viewer->removeShape("arm",0);
	drawPoint("arm",solvedTransform * armPosition,1,0,0);
}

/*Registers to the tfPublisher. He will then subscribe to pointcloud, do smoothing and sends back te result to the given callback.*/
void askForCloud()
{
	std::cout << "[dobotCalibration]: Asking for new cloud!" << std::endl;
	detection_msgs::SetString srv;
	srv.request.msg = "/dobotCalibration/getPointcloud";
	if(!registerPointcloudSubscriber.call(srv))
	{
		std::cout << "[dobotCalibration]: Failed to ask for cloud!" << std::endl;
	}
}

bool askPointcloudCB(detection_msgs::SetPointcloud::Request &req, detection_msgs::SetPointcloud::Response &res)
{
	try{
		std::cout << "Cloud from frame: "<< req.cloud.header.frame_id << std::endl;
		geometry_msgs::TransformStamped tS;
		tS = tfBuffer.lookupTransform("world",req.cloud.header.frame_id,ros::Time(0));
		sensor_msgs::PointCloud2 other;
		tf2::doTransform(req.cloud,other,tS);
		pcl::PCLPointCloud2 cloud2;
		pcl_conversions::toPCL(other,cloud2);
		pcl::fromPCLPointCloud2(cloud2,*sampledCloud);

		viewer->removeShape("info",0);
		if(dobotPoints.size() < 3)
		{
			viewer->addText("Select a point in the point cloud!",0,70,15,1.0,1.0,1.0,"info",0);
		}else{
			viewer->addText("A = Add a new point; B = Send to tfPublisher; START = Save all tfPublisher frames!",0,70,15,1.0,1.0,1.0,"info",0);
		}
	}catch(tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
		return true;
	}
	updatedCloud = true;
	return true;
}

int findClosePoint(std::vector<tf2::Vector3> points, double maxDistance, tf2::Vector3 other)
{
	int nearest = -1;
	double smallestDist = maxDistance;
	for(size_t i = 0; i < points.size(); i++)
	{
		double d = points.at(i).distance(other);
		if((nearest == -1) || (d < smallestDist))
		{
			smallestDist = d;
			nearest = i;
		}
	}

	if(smallestDist < maxDistance)
	{
		return nearest;
	}
	return -1;
}
/*Moves the dobot arm to a given point. It will include the arm length offset and expects points in world coordinates.*/
void commandDobotArm(tf2::Vector3 goal)
{
	ROS_INFO("Moving Dobot!");
	if(!allowDobotMove) return;

	tf2::Vector3 graspPoint;
	if(arm.hasLinearRail())
	{
		double railPos;
		if(arm.convertToDobotPointWithL(goal,graspPoint,railPos,"world"))//Converts from any frame to dobot coordinate frame. Converts from meters to mm.
		{
			arm.moveArmToPositionWithL(graspPoint,0,railPos);
		}
	}else{
		if(arm.convertToDobotPoint(goal,graspPoint,"world"))//Converts from any frame to dobot coordinate frame. Converts from meters to mm.
		{
			arm.moveArmToPosition(graspPoint,0);
		}
	}
}

void mouseClickEvent(const pcl::visualization::PointPickingEvent &event, void*)
{
	if(event.getPointIndex() == -1)
	{
		ROS_ERROR("No valid point selected!");
		return;
	}

	float x,y,z;
	event.getPoint(x,y,z);
	tf2::Vector3 point(x,y,z);

	if(clickMoveMode)
	{
		commandDobotArm(point);
		return;
	}

	int selectedPoint = findClosePoint(worldPoints,0.008,point);
	if(selectedPoint == -1)
	{
		if(selectCloudPoint)
		{
			worldPoints.push_back(point);
			drawPoints();
			viewer->removeShape("info",0);
			viewer->addText("Move Dobot into same position and press A!",0,70,15,1.0,1.0,1.0,"info",0);
			selectCloudPoint = false;
		}
	}else{
		selectCloudPoint = true;
		if(worldPoints.size() > size_t(selectedPoint)) worldPoints.erase(worldPoints.begin() + selectedPoint);
		if(dobotPoints.size() > size_t(selectedPoint)) dobotPoints.erase(dobotPoints.begin() + selectedPoint);
		if(transPoints.size() > size_t(selectedPoint)) transPoints.erase(transPoints.begin() + selectedPoint);
		drawPoints();
	}
}

/**
 * Will be called in a frequent time with respect to UPDATE_RATE. Reads the last send joystick commands and reacts to them.
 * @param t
 */
void timerCallback (const ros::TimerEvent& t)
{
	if (!joyMessageValid) return;

	double currentSpeed = JOY_SPEED_NORMAL;
	double speedupManualControl = 1.0;
	if(joyMessage->buttons[4] == 1)//LB Button: Change of movement speed
	{
		currentSpeed = JOY_SPEED_SLOW;
		speedupManualControl = 0.25;
	}
	else if(joyMessage->buttons[5] == 1)//RB Button: Change of movement speed
	{
		currentSpeed = JOY_SPEED_FAST;
		speedupManualControl = 5.0;
	}
	//Manually move dobot!
	if(joyMessage->axes[5] < 0)//LT Button: Change of corrdinate frame scale (visual)
	{
		if(allowDobotMove)
		{
			arm.moveArmToPosition(armPositionDobotFrame + tf2::Vector3(0,0,joyMessage->axes[5]*speedupManualControl),0,dobot::PtpMode::MOVL_XYZ);
		}
	}
	if(joyMessage->axes[2] < 0)//RT Button: Change of corrdinate frame scale (visual)
	{
		if(allowDobotMove)
		{
			arm.moveArmToPosition(armPositionDobotFrame + tf2::Vector3(0,0,-joyMessage->axes[2]*speedupManualControl),0,dobot::PtpMode::MOVL_XYZ);
		}
	}
	if(std::fabs(joyMessage->axes[6]) > 0.1)//RT Button: Change of corrdinate frame scale (visual)
	{
		if(allowDobotMove)
		{
			if(arm.hasLinearRail())
			{
				commandedRailPosDobotFrame += joyMessage->axes[6]*speedupManualControl*5.0;
				commandedRailPosDobotFrame = std::max(-1000.0,std::min(commandedRailPosDobotFrame,0.0));
				std::cout << "Rail goal= " << commandedRailPosDobotFrame << std::endl;
				arm.moveArmToPositionWithL(armPositionDobotFrame,0,-commandedRailPosDobotFrame,dobot::PtpMode::MOVL_XYZ);
			}else{
				arm.moveArmToPosition(armPositionDobotFrame + tf2::Vector3(0,joyMessage->axes[6]*speedupManualControl,0),0,dobot::PtpMode::MOVL_XYZ);
			}
		}
	}
	if(std::fabs(joyMessage->axes[7]) > 0.1)//RT Button: Change of corrdinate frame scale (visual)
	{
		if(allowDobotMove)
		{
			arm.moveArmToPosition(armPositionDobotFrame + tf2::Vector3(joyMessage->axes[7]*speedupManualControl,0,0),0,dobot::PtpMode::MOVL_XYZ);
		}
	}

	if(joyMessage->buttons[0] == 1)//A Button
	{
		if(!buttonsProcessed[0])
		{
			buttonsProcessed[0] = true;
			if(!selectCloudPoint)
			{
				dobotPoints.push_back(armPosition);
				drawPoints();

				viewer->removeShape("info",0);
				if(dobotPoints.size() < 3)
				{
					viewer->addText("Select a point in the point cloud!",0,70,15,1.0,1.0,1.0,"info",0);
				}else{
					viewer->addText("A = Add a new point; B = Send to tfPublisher; START = Save all tfPublisher frames!",0,70,15,1.0,1.0,1.0,"info",0);
				}
				viewer->removeShape("info2",0);
				viewer->addText("Number of Points: " + std::to_string(dobotPoints.size()),0,50,15,1.0,1.0,1.0,"info2",0);
				viewer->removeShape("info3",0);
				viewer->addText("Location: [" + std::to_string(solvedTransform.getOrigin().x()) + ", " + std::to_string(solvedTransform.getOrigin().y())
						+ ", " + std::to_string(solvedTransform.getOrigin().z()) + "]",0,30,15,1.0,1.0,1.0,"info3",0);
				tf2::Vector3 rot = solvedTransform.getRotation().getAxis();
				rot *= (180.0/M_PI);
				viewer->removeShape("info4",0);
				viewer->addText("Rotation: [" + std::to_string(rot.x()) + ", " + std::to_string(rot.y())
							+ ", " + std::to_string(rot.z()) + "]",0,10,15,1.0,1.0,1.0,"info4",0);
				selectCloudPoint = true;

				if(worldPoints.size() >= 4)
				{
					minimizeWithEigen();
					transPoints.clear();
					for(size_t i = 0; i < dobotPoints.size(); i++)
					{
						tf2::Vector3 transformedPoint = solvedTransform * dobotPoints.at(i);
						transPoints.push_back(transformedPoint);
					}
					drawPoints();
				}
			}
		}
	}else{
		buttonsProcessed[0] = false;
	}

	if(joyMessage->buttons[2] == 1)//X Button
	{
		if(!buttonsProcessed[2])
		{
			buttonsProcessed[2] = true;
			commandDobotArm(currentTransform.getOrigin());
		}
	}else{
		buttonsProcessed[2] = false;
	}
	if(joyMessage->buttons[6] == 1)//Back Button
	{
		if(!buttonsProcessed[6])
		{
			buttonsProcessed[6] = true;
			ROS_INFO("Clear all frames.");
			worldPoints.clear();
			dobotPoints.clear();
			transPoints.clear();
			selectCloudPoint = true;
			drawPoints();
		}
	}else{
		buttonsProcessed[6] = false;
	}
	if((joyMessage->buttons[7] == 1) || (joyMessage->buttons[1] == 1))//Start Button
	{
		if(!buttonsProcessed[7])
		{
			buttonsProcessed[7] = true;
			detection_msgs::SetFrame::Request req;
			detection_msgs::SetFrame::Response res;
			req.name = dobotFrame;
			req.t.translation.x = solvedTransform.getOrigin().x();
			req.t.translation.y = solvedTransform.getOrigin().y();
			req.t.translation.z = solvedTransform.getOrigin().z();
			req.t.rotation.x = solvedTransform.getRotation().x();
			req.t.rotation.y = solvedTransform.getRotation().y();
			req.t.rotation.z = solvedTransform.getRotation().z();
			req.t.rotation.w = solvedTransform.getRotation().w();
			setFrame.call(req,res);
			ROS_INFO("Sending transform to server!");
		}
	}else{
		buttonsProcessed[7] = false;
	}
	if(joyMessage->buttons[1] == 1)//B Button
	{
		if(!buttonsProcessed[1])
		{
			clickMoveMode = !clickMoveMode;
			viewer->removeShape("info",0);
			if(clickMoveMode)
			{
				viewer->addText("Click Move Mode enabled! Shift + Left mouse to direcly command dobot!",0,70,15,1.0,1.0,1.0,"info",0);
			}else{
				if(dobotPoints.size() < 3)
				{
					viewer->addText("Select a point in the point cloud!",0,70,15,1.0,1.0,1.0,"info",0);
				}else{
					viewer->addText("A = Add a new point; B = Send to tfPublisher; START = Save all tfPublisher frames!",0,70,15,1.0,1.0,1.0,"info",0);
				}
			}
		}
	}else{
		buttonsProcessed[1] = false;
	}
	if((joyMessage->buttons[3] == 1) && nh)//Y Button: Records a new pointcloud after pointcloud callback function has been called.
	{
		if(!buttonsProcessed[3])
		{
			viewer->removeShape("info",0);
			viewer->addText("Requesting new PointCloud. Please wait!",0,70,15,1.0,1.0,1.0,"info",0);
			std::cout << "[dobotCalibration]: Getting new cloud!" << std::endl;
			askForCloud();
			buttonsProcessed[3] = true;
		}
	}else{
		buttonsProcessed[3] = false;
	}
	if(joyMessage->buttons[10] == 1 || joyMessage->buttons[9] == 1) //Used to snap into the neares point
	{
		if(!buttonsProcessed[10])
		{
			tf2::Vector3 currentPosition = currentTransform.getOrigin();
			tf2::Vector3 loc(0,0,0);
			tf2::Quaternion rot(0,0,0,1);
			size_t id;
			if(cloud::getClosestPoint(currentPosition,sampledCloud,id,loc))
			{
				currentTransform.setOrigin(loc);
			}
			buttonsProcessed[10] = true;
		}
	}else{
		buttonsProcessed[10] = false;
	}

	//Used for moving and rotating the world frame.
	tf2::Vector3 deltaLocation;
	deltaLocation.setX (currentSpeed * (joyMessage->axes[1]));
	deltaLocation.setY (currentSpeed * (joyMessage->axes[0]));
	deltaLocation.setZ (currentSpeed * joyMessage->axes[4]);

	currentTransform.setOrigin (currentTransform.getOrigin () + deltaLocation);
}

int main (int argc, char* argv[])
{
	if(argc < 4)
	{
		std::cout << "Usage: rosrun detection DobotCalibration DOBOT_NAME DOBOT_FRAME CAMERA_FRAME [ALLOW_PTP_CMD=false]\n";
		return -1;
	}
	frameName = argv[1];
	dobotFrame = argv[2];
	cameraName = argv[3];

	if(argc > 4)
	{
		allowDobotMove = std::string(argv[4]) == "true";
		std::cout << "Move commands are " << (allowDobotMove?"enabled":"disabled") << std::endl;
	}

	ros::init (argc, argv, "dobotCalibration");
	nh = std::make_shared<ros::NodeHandle>();
	tf2_ros::TransformListener tfListener(tfBuffer);
	sampledCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	arm.initArm(nh,frameName);

	//Creating the PointCloud Viewer
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>();
	viewer->setBackgroundColor (0.1, 0.1, 0.1, 0);
	viewer->setWindowName ("dobotCalibration");
	viewer->initCameraParameters ();
	viewer->addText("Select a point in the point cloud!",0,70,15,1.0,1.0,1.0,"info",0);
	viewer->registerPointPickingCallback(mouseClickEvent);
	drawOrigin();

	//Subscribing...
	ros::Subscriber joySub = nh->subscribe<sensor_msgs::Joy> ("/joy", 2, joyCallback);
	ros::Subscriber railSub = nh->subscribe<geometry_msgs::Pose> ("/" + frameName + "/arm/Pose", 1, armCallback);
	ros::Subscriber armSub = nh->subscribe<std_msgs::Float64> ("/" + frameName + "/linear_rail/pos", 1, railCallback);
	setFrame = nh->serviceClient<detection_msgs::SetFrame>("/tfPublisher/setFrame");
	saveFrames = nh->serviceClient<std_srvs::Empty>("/tfPublisher/save");
	registerPointcloudSubscriber =  nh->serviceClient<detection_msgs::SetString>("/" + cameraName + "/askForPointcloud");
	ros::ServiceServer getPointCloudServer = nh->advertiseService("/dobotCalibration/getPointcloud",askPointcloudCB);
	askForCloud();

	//Creates a timer which will frequently check the last joystick command and reacts to it.
	ros::Timer timer = nh->createTimer (ros::Duration (1.0F / 4.0), timerCallback);

	recording = true;
	ros::Rate r(UPDATE_RATE);
	ros::Time::sleepUntil(ros::Time(1));

	try{
		geometry_msgs::TransformStamped tS;
		tS = tfBuffer.lookupTransform("world",dobotFrame,ros::Time(0));
		tf2::Stamped<tf2::Transform> t;
		tf2::convert(tS,t);
		solvedTransform = t;
	}
	catch(tf2::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		return -1;
	}

	std::cout << "[dobotCalibration]: Starting loop!\n";
	while (ros::ok())
	{
		if(updatedCloud) //Called if all clouds have been recieved.
		{
			std::cout << "[dobotCalibration]: Showing new cloud!\n";
			viewer->removePointCloud("current",0);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (sampledCloud);
			viewer->addPointCloud<pcl::PointXYZRGB> (sampledCloud, rgb, "current");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current");
			viewer->resetCamera();
			updatedCloud = false;
		}

		geometry_msgs::TransformStamped tS;
		tS = tfBuffer.lookupTransform(dobotFrame,frameName + "_tool",ros::Time(0));
		tf2::Stamped<tf2::Transform> transform;
		tf2::convert(tS,transform);
		armPosition = transform.getOrigin();

		drawOrigin();
		viewer->spinOnce();
		ros::spinOnce ();
		r.sleep();
	}
	std::cout << "[dobotCalibration]: Quit." << std::endl;
	return 0;
}
