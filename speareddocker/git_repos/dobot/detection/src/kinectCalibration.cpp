/**
 * kinectCalibration.cpp
 * Used to find out the pose of the second kinect camera. The same set of points has to be calculated in both cameras, then the pose is estimated
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
#include <pcl/common/transforms.h>
#include <pcl/common/poses_from_matches.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <detection_msgs/SetFrame.h>
#include <std_srvs/Empty.h>

#include "../include/cloud.h"
#include "../include/medianFilter.h"

const float UPDATE_RATE = 30.0;
//Definition of speeds for moving the frame.
const double JOY_SPEED_NORMAL = 0.005;
const double JOY_SPEED_SLOW = JOY_SPEED_NORMAL * 0.025;
const double JOY_SPEED_FAST = JOY_SPEED_NORMAL * 5;
const std::string CLOUD_FRAME = "world";
//Number of samples which will be combined into one shot. More samples reduce noise, but cost more computation time.
const size_t MAX_NUM_SAMPLES = 16;
//Position of cursor
tf2::Transform currentTransform = tf2::Transform(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));
tf2::Transform solvedTransform = tf2::Transform(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));

ros::Subscriber cloudSub;
ros::Subscriber slaveCloudSub;
std::shared_ptr<ros::NodeHandle> nh;
tf2_ros::Buffer tfBuffer;

pcl::visualization::PCLVisualizer viewer;
pcl::visualization::PCLVisualizer slaveViewer;
pcl::visualization::PCLVisualizer resultViewer;

pcl::PointCloud<cloud::PointType>::Ptr sampledCloud;
pcl::PointCloud<cloud::PointType>::Ptr sampledSlaveCloud;
pcl::PointCloud<cloud::PointType>::Ptr resultCloud;
ros::ServiceClient saveFrames;
ros::ServiceClient setFrame;

std::string frameName = "kinect2_xavier_link";

cloud::MedianFilter medianFilter(false);
cloud::MedianFilter slaveMedianFilter(false);
bool recording = false;
bool updatedCloud = false;
bool updatedSlaveCloud = false;
bool init = true;
bool masterHasToSelect = true;
double coordinateScale = 0.05;

std::vector<tf2::Vector3> worldPoints;
std::vector<tf2::Vector3> slavePoints;
std::vector<tf2::Vector3> transPoints;
std::vector<std::string> drawnPoints;

sensor_msgs::JoyConstPtr joyMessage;
bool joyMessageDitry = false;

//Buttons should be only processed once. This array sets a button to true, if it already has been processed.
bool buttonsProcessed[12] = {false,false,false,false,false,false,false,false,false,false,false,false};

/**
 * Callback if new pointcloud has been recorded.
 * @param pointcloud The pointcloud will be only applied on startup, or after y button has been pressed.
 */
void pointcloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
	if(!recording) return;
	if(msg->width == 0) return;

	//Transform cloud into world frame and save it into samples
	sensor_msgs::PointCloud2 other;
	geometry_msgs::TransformStamped tS;
	pcl::PointCloud<cloud::PointType>::Ptr sample = boost::make_shared<pcl::PointCloud<cloud::PointType> >();
	try{
		tS = tfBuffer.lookupTransform("world",msg->header.frame_id,ros::Time(0));
		tf2::doTransform(*msg,other,tS);
		pcl::fromROSMsg(other,*sample);
	}catch(tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
		return;
	}

	medianFilter.push_back(sample);
	std::cout << "[dobotCalibration]: Recorded " << medianFilter.size() << " of " << MAX_NUM_SAMPLES << " for main!"<< std::endl;
	if(medianFilter.size() >= MAX_NUM_SAMPLES)
	{
		updatedCloud = true;
		cloudSub.shutdown();
	}
}

void slavePointcloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
	if(!recording) return;
	if(msg->width == 0) return;

	pcl::PointCloud<cloud::PointType>::Ptr sample = boost::make_shared<pcl::PointCloud<cloud::PointType> >();
	pcl::fromROSMsg(*msg,*sample);
	slaveMedianFilter.push_back(sample);
	std::cout << "[dobotCalibration]: Recorded " << slaveMedianFilter.size() << " of " << MAX_NUM_SAMPLES << " for slave!"<< std::endl;
	if(slaveMedianFilter.size() >= MAX_NUM_SAMPLES)
	{
		updatedSlaveCloud = true;
		slaveCloudSub.shutdown();
	}
}
/**
 * Joystick callback.
 * @param joy
 */
void joyCallback (const sensor_msgs::JoyConstPtr &joy)
{
	joyMessage = joy;
	joyMessageDitry = true;
}
//Remove all custom points from the viewer
void removePoints()
{
	for(auto s : drawnPoints)
	{
		viewer.removeShape(s,0);
	}
	for(auto s : drawnPoints)
	{
		slaveViewer.removeShape(s,0);
	}
}
//Draws a point at the given location in the 3d viewer
void drawPoint(std::string id, tf2::Vector3 pos, double r, double g, double b, pcl::visualization::PCLVisualizer &v)
{
	pcl::PointXYZ p;
	p.x = pos.x();
	p.y = pos.y();
	p.z = pos.z();
	v.addSphere(p,0.005,r,g,b,id,0);
}
//Draws all custom points in the 3d viewers
void drawPoints()
{
	removePoints();
	for(size_t i = 0; i < slavePoints.size(); i++)
	{
		std::string id = std::to_string(i) + "_dobot";
		drawnPoints.push_back(id);
		drawPoint(id,slavePoints.at(i),0,1,0,slaveViewer);
	}

	for(size_t i = 0; i < worldPoints.size(); i++)
	{
		std::string id = std::to_string(i) + "_world";
		drawnPoints.push_back(id);
		drawPoint(id,worldPoints.at(i),0,1,0,viewer);
	}

	for(size_t i = 0; i < transPoints.size(); i++)
	{
		std::string id = std::to_string(i) + "_trans";
		drawnPoints.push_back(id);
		drawPoint(id,transPoints.at(i) + tf2::Vector3(0.0,1.0,0.0),0,0,1,viewer);
	}
}
//finds the closest point in a set of points.
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
//Given two sets of points, the tramsform is searched.
void minimizeWithEigen()
{
	Eigen::MatrixXd start,end;
	start.conservativeResize(3,slavePoints.size());
	end.conservativeResize(3,slavePoints.size());

	for(size_t i = 0; i < slavePoints.size(); i++)
	{
		//Adding offset of 1m on y axis
		end.col(i) = Eigen::Vector3d(worldPoints.at(i).x(),worldPoints.at(i).y() - 1.0,worldPoints.at(i).z());
		start.col(i) = Eigen::Vector3d(slavePoints.at(i).x(),slavePoints.at(i).y(),slavePoints.at(i).z());
	}
	Eigen::Matrix<double,4,4> solution = Eigen::umeyama(start,end);
	std::cout << solution.matrix() << std::endl;
	Eigen::Affine3d t;
	t = solution;
	tf2::convert(t,solvedTransform);
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
	viewer.removeCoordinateSystem("origin",0);
	viewer.addCoordinateSystem (coordinateScale, affine,"origin", 0);
}
//If a new point has been selected, check what to do next
void newPointSelected(tf2::Vector3 point)
{
	if(masterHasToSelect)
	{
		worldPoints.push_back(point);
		drawPoints();
		viewer.removeShape("info",0);
		slaveViewer.addText("Select a point in the point cloud using shift + Left Mouse!",0,70,15,1.0,1.0,1.0,"info",0);
		masterHasToSelect = false;
	}else{

		slavePoints.push_back(point);
		drawPoints();

		slaveViewer.removeShape("info",0);
		if(slavePoints.size() < 3) //Update text
		{
			viewer.addText("Select a point in the point cloud using shift + Left Mouse!",0,70,15,1.0,1.0,1.0,"info",0);
		}else{
			viewer.addText("A = Add a new point; B = Send to tfPublisher; START = Save all tfPublisher frames!",0,70,15,1.0,1.0,1.0,"info",0);
		}
		viewer.removeShape("info2",0);
		viewer.addText("Number of Points: " + std::to_string(slavePoints.size()),0,50,15,1.0,1.0,1.0,"info2",0);
		viewer.removeShape("info3",0);
		viewer.addText("Location: [" + std::to_string(solvedTransform.getOrigin().x()) + ", " + std::to_string(solvedTransform.getOrigin().y())
				+ ", " + std::to_string(solvedTransform.getOrigin().z()) + "]",0,30,15,1.0,1.0,1.0,"info3",0);
		tf2::Vector3 rot = solvedTransform.getRotation().getAxis();
		rot *= (180.0/M_PI);
		viewer.removeShape("info4",0);
		viewer.addText("Rotation: [" + std::to_string(rot.x()) + ", " + std::to_string(rot.y())
					+ ", " + std::to_string(rot.z()) + "]",0,10,15,1.0,1.0,1.0,"info4",0);
		masterHasToSelect = true;

		if(worldPoints.size() >= 4) //Okay, we have a new slave and master point. Now we can recalculate the camera pose
		{
			minimizeWithEigen();
			transPoints.clear();
			for(size_t i = 0; i < slavePoints.size(); i++)
			{
				tf2::Vector3 transformedPoint = solvedTransform * slavePoints.at(i);
				transPoints.push_back(transformedPoint);
			}
			drawPoints();
			Eigen::Affine3d t;
			geometry_msgs::Transform gT;
			tf2::convert(solvedTransform,gT);
			t = tf2::transformToEigen(gT);
			Eigen::Affine3f affine = t.cast<float>();
			pcl::PointCloud<cloud::PointType>::Ptr cloud = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
			pcl::transformPointCloud(*sampledSlaveCloud,*cloud,affine,true);
			*resultCloud = *sampledCloud;
			*resultCloud += *cloud;

			resultViewer.removeAllPointClouds(0);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (resultCloud);
			resultViewer.addPointCloud<pcl::PointXYZRGB> (resultCloud, rgb, "current");
			resultViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current");
		}
	}
}
//Click event from the source pointcloud frame
void masterClickEvent(const pcl::visualization::PointPickingEvent &event, void*)
{
	if(event.getPointIndex() == -1)
	{
		ROS_ERROR("No valid point selected!");
		return;
	}
	float x,y,z;
	event.getPoint(x,y,z);
	tf2::Vector3 point(x,y,z);
	int selectedPoint = findClosePoint(worldPoints,0.005,point);
	if(selectedPoint == -1)
	{
		if(!masterHasToSelect)
		{
			ROS_ERROR("Please select point in slave viewer!");
			return;
		}
		newPointSelected(point);
	}else{
		masterHasToSelect = true;
		if(worldPoints.size() > size_t(selectedPoint)) worldPoints.erase(worldPoints.begin() + selectedPoint);
		if(slavePoints.size() > size_t(selectedPoint)) slavePoints.erase(slavePoints.begin() + selectedPoint);
		if(transPoints.size() > size_t(selectedPoint)) transPoints.erase(transPoints.begin() + selectedPoint);
		drawPoints();
	}
}
//Click event from the target pointcloud frame
void slaveClickEvent(const pcl::visualization::PointPickingEvent &event, void*)
{
	if(event.getPointIndex() == -1)
	{
		ROS_ERROR("No valid point selected!");
		return;
	}
	float x,y,z;
	event.getPoint(x,y,z);
	tf2::Vector3 point(x,y,z);
	int selectedPoint = findClosePoint(slavePoints,0.005,point);
	if(selectedPoint == -1)
	{
		if(masterHasToSelect)
		{
			ROS_ERROR("Please select point in master viewer!");
			return;
		}
		newPointSelected(point);
	}else{
		masterHasToSelect = true;
		if(worldPoints.size() > size_t(selectedPoint)) worldPoints.erase(worldPoints.begin() + selectedPoint);
		if(slavePoints.size() > size_t(selectedPoint)) slavePoints.erase(slavePoints.begin() + selectedPoint);
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
	if (!joyMessageDitry) return;

	if(joyMessage->buttons[1] == 1)//B Button
	{
		if(!buttonsProcessed[1])
		{
			buttonsProcessed[1] = true;

			detection_msgs::SetFrame::Request req;
			detection_msgs::SetFrame::Response res;
			req.name = frameName;
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
		buttonsProcessed[1] = false;
	}

	if(joyMessage->buttons[6] == 1)//Back Button
	{
		if(!buttonsProcessed[6])
		{
			buttonsProcessed[6] = true;
			ROS_INFO("Clear all frames.");
			worldPoints.clear();
			slavePoints.clear();
			transPoints.clear();
			masterHasToSelect = true;
			drawPoints();
		}
	}else{
		buttonsProcessed[6] = false;
	}
	if(joyMessage->buttons[7] == 1)//Start Button
		{
			if(!buttonsProcessed[7])
			{
				buttonsProcessed[7] = true;
				detection_msgs::SetFrame::Request req;
				detection_msgs::SetFrame::Response res;
				req.name = frameName;
				req.t.translation.x = solvedTransform.getOrigin().x();
				req.t.translation.y = solvedTransform.getOrigin().y();
				req.t.translation.z = solvedTransform.getOrigin().z();
				req.t.rotation.x = solvedTransform.getRotation().x();
				req.t.rotation.y = solvedTransform.getRotation().y();
				req.t.rotation.z = solvedTransform.getRotation().z();
				req.t.rotation.w = solvedTransform.getRotation().w();
				setFrame.call(req,res);
				ROS_INFO("Sending transform to server!");

				ROS_INFO("Saving %s",frameName.c_str());
				std_srvs::Empty srv;
				saveFrames.call(srv);
			}
		}else{
			buttonsProcessed[7] = false;
		}
	if((joyMessage->buttons[3] == 1) && nh)//Y Button: Records a new pointcloud after pointcloud callback function has been called.
	{
		if(!buttonsProcessed[3])
		{
			std::cout << "[dobotCalibration]: Getting new cloud!" << std::endl;
			recording = true;
			medianFilter.clear();
			slaveMedianFilter.clear();
			cloudSub = nh->subscribe <sensor_msgs::PointCloud2> ("/kinect2_jetson/hd/points", 2, pointcloudCallback);
			slaveCloudSub = nh->subscribe <sensor_msgs::PointCloud2> ("/kinect2_xavier/hd/points", 2, slavePointcloudCallback);
			buttonsProcessed[3] = true;
		}
	}else{
		buttonsProcessed[3] = false;
	}
}

int main (int argc, char* argv[])
{
	if(argc == 1)
	{
		std::cout << "Usage: rosrun detection kinectCalibration [FRAME=" << frameName << "]\n";
	}else{
		frameName = argv[1];
	}

	ros::init (argc, argv, "kinectCalibration");
	nh = std::make_shared<ros::NodeHandle>();
	tf2_ros::TransformListener tfListener(tfBuffer);

	sampledCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	sampledSlaveCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	resultCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

	//Creating the PointCloud Viewer
	viewer.setBackgroundColor (0.1, 0.1, 0.1, 0);
	viewer.setWindowName ("mainViewer");
	viewer.initCameraParameters ();
	viewer.addText("Select a point in the point cloud using shift + Left Mouse!",0,70,15,1.0,1.0,1.0,"info",0);

	slaveViewer.setBackgroundColor (0.1, 0.1, 0.1, 0);
	slaveViewer.setWindowName ("slaveViewer");
	slaveViewer.initCameraParameters ();

	resultViewer.setBackgroundColor (0.1, 0.1, 0.1, 0);
	resultViewer.setWindowName ("resultViewer");
	resultViewer.initCameraParameters ();

	drawOrigin();

	//Subscribing...
	ros::Subscriber joySub = nh->subscribe<sensor_msgs::Joy> ("/joy", 2, joyCallback);
	cloudSub = nh->subscribe <sensor_msgs::PointCloud2> ("/kinect2_jetson/hd/points", 2, pointcloudCallback);
	slaveCloudSub = nh->subscribe <sensor_msgs::PointCloud2> ("/kinect2_xavier/hd/points", 2, slavePointcloudCallback);
	setFrame = nh->serviceClient<detection_msgs::SetFrame>("/tfPublisher/setFrame");
	saveFrames = nh->serviceClient<std_srvs::Empty>("/tfPublisher/save");

	//Creates a timer which will frequently check the last joystick command and reacts to it.
	ros::Timer timer = nh->createTimer (ros::Duration (1.0F / 60.0), timerCallback);

	recording = true;
	ros::Rate r(UPDATE_RATE);
	ros::Time::sleepUntil(ros::Time(1));

	try{
		geometry_msgs::TransformStamped tS;
		tS = tfBuffer.lookupTransform("world",frameName,ros::Time(0));
		tf2::Stamped<tf2::Transform> t;
		tf2::convert(tS,t);
		solvedTransform = t;
	}
	catch(tf2::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		//return -1;
	}

	std::cout << "[dobotCalibration]: Starting loop!\n";
	while (ros::ok())
	{
		if(updatedCloud) //Called if all clouds have been recieved.
		{
			std::cout << "[dobotCalibration]: Starting median filter:\n";
			medianFilter.compute(sampledCloud);

			std::cout << "[dobotCalibration]: Showing new cloud!\n";
			viewer.removePointCloud("current",0);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (sampledCloud);
			viewer.addPointCloud<pcl::PointXYZRGB> (sampledCloud, rgb, "current");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current");
			viewer.resetCamera();
			viewer.registerPointPickingCallback(masterClickEvent);
			updatedCloud = false;
		}
		if(updatedSlaveCloud) //Called if all clouds have been recieved.
		{
			std::cout << "[dobotCalibration]: Starting slaveMedian filter:\n";
			slaveMedianFilter.compute(sampledSlaveCloud);

			std::cout << "[dobotCalibration]: Showing new slave cloud!\n";
			slaveViewer.removePointCloud("current",0);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (sampledSlaveCloud);
			slaveViewer.addPointCloud<pcl::PointXYZRGB> (sampledSlaveCloud, rgb, "current");
			slaveViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current");
			slaveViewer.resetCamera();
			slaveViewer.registerPointPickingCallback(slaveClickEvent);
			updatedSlaveCloud = false;
		}
		if(medianFilter.size() >= MAX_NUM_SAMPLES && slaveMedianFilter.size() >= MAX_NUM_SAMPLES)
		{
			recording = false;
		}
		drawOrigin();
		viewer.spinOnce();
		slaveViewer.spinOnce();
		resultViewer.spinOnce();
		ros::spinOnce ();
		r.sleep();
	}
	std::cout << "[dobotCalibration]: Quit." << std::endl;
	return 0;
}
