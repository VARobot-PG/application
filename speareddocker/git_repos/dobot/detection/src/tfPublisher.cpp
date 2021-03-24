/**
 * tfPublisher.cpp
 *
 *	This node manages all static coordinate frames in the dobot scenario. The transforms are required to convert camera points into the dobot coordinate frame.
 *	It can be calibrated using tfPublisherClient, kinectCalibration and dobotCalibration, which offer a user interface.
 *	This node only offers services, which might be helpfull for finetuning while the system is running, or adding new coordinate frames.
 *	The coordinate frames are saved in a .txt file.
 *  Created on: Mai 20, 2019
 *      Author: philipf
 *
 */
#include <iostream>
#include <string>
#include <fstream>
#include <queue>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../include/medianFilter.h"

#include <tf_helper/tfHelper.h>

#include <stdlib.h>
#include <geometry_msgs/Vector3.h>
#include <detection_msgs/AddFrame.h>
#include <detection_msgs/SetFrame.h>
#include <detection_msgs/GetFrames.h>
#include <detection_msgs/SetPointcloud.h>
#include <detection_msgs/SetString.h>
#include <std_srvs/Empty.h>

struct Frame
{
	Frame(std::string gName="") : name(gName)
	{
		transform = tf2::Transform(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));
	}
	std::string name;
	tf2::Transform transform;
	bool dirty = true;
};

const float UPDATE_RATE = 30.0;
std::string CLOUD_FRAME = "kinect2_jetson_link";

boost::shared_ptr<ros::NodeHandle> nh;

bool init = true;
std::vector<Frame> frames;
tf2_ros::Buffer tfBuffer;

/**
 * Loads a transform from /settings/cameraFrame.settings which can be stored by pressing the start button.
 * @param t Loaded transform will be placed into t.
 * @return Returns true if cameraFrame.settings exists.
 */
bool loadTransforms(std::vector<Frame>& frame_vec, std::string fileName)
{
	std::cout << "[tfPublisher]: Loading frames: " << fileName << std::endl;
	frame_vec.clear();
	std::ifstream file(fileName);
	if(file.fail())
	{
		std::cout << "[tfPublisher]: Failed to load file: " << fileName << "\n";
		return false;
	}
	std::string text;
	tf2::Vector3 loc(0,0,0);
	tf2::Quaternion rot(0,0,0,0);
	tf2::Transform t;
	Frame currentF;
	getline(file,text);
	while(text.size() > 0)
	{
		std::stringstream ss(text);
		int i = 0;
		while(ss.good())
		{
			std::string substr;
			getline(ss,substr,',');
			switch(i)
			{
				case 0:
				{
					double value = std::stof(substr.c_str());
					loc.setX(value); break;
				}
				case 1:
				{
					double value = std::stof(substr.c_str());
					loc.setY(value); break;
				}
				case 2:
				{
					double value = std::stof(substr.c_str());
					loc.setZ(value); break;
				}

				case 3:
				{
					double value = std::stof(substr.c_str());
					rot.setX(value); break;
				}
				case 4:
				{
					double value = std::stof(substr.c_str());
					rot.setY(value); break;
				}
				case 5:
				{
					double value = std::stof(substr.c_str());
					rot.setZ(value); break;
				}
				case 6:
				{
					double value = std::stof(substr.c_str());
					rot.setW(value); break;
				}
				case 7:
				{
					t.setOrigin(loc);
					t.setRotation(rot);
					currentF.transform = t;
					currentF.name = substr;
					currentF.dirty = true;
					frame_vec.push_back(currentF);
					break;
				}
				default:;
			}
			i++;
		}
		getline(file,text);
	}
	return true;
}
/**
 * Stores the given transform and frame_id into cameraFrame.settings, which can be loaded via loadTransform.
 * @param t Transform to store.
 * @return Returns true, if cameraFrame.settings cloud be succesfully created.
 */
bool storeTransforms(std::vector<Frame> frame_vec, std::string fileName)
{
	std::ofstream file;
	std::cout << "[tfPublisher]: Saving frames: " << fileName << std::endl;
	file.open(fileName.c_str());
	if(file.fail())
	{
		return false;
	}
	for(size_t i = 0; i < frame_vec.size(); i++)
	{
		tf2::Transform t = frame_vec.at(i).transform;
		file << t.getOrigin().x() << "," << t.getOrigin().y() << "," << t.getOrigin().z() << ","
				<< t.getRotation().x() << "," << t.getRotation().y() << "," << t.getRotation().z() << "," << t.getRotation().w() <<
				","<< frame_vec.at(i).name << std::endl;
	}
	file.close();
	return true;
}
/*Adds a new coordinate frame to the world*/
bool addNewFrameCB(detection_msgs::AddFrame::Request &req, detection_msgs::AddFrame::Response &res)
{
	if(req.name == "/world")
	{
		ROS_ERROR("Can not add ’/world’ frame!");
		return false;
	}

	if(req.name.find(",") != std::string::npos)
	{
		ROS_ERROR("Character ’,’ is not allowed as a frame name!");
		return false;
	}
	Frame f(req.name);
	if(req.useXYZ)
	{
		f.transform.setOrigin(tf2::Vector3(req.x,req.y,req.z));
	}
	if(req.useRotation)
	{
		f.transform.setRotation(tf2::Quaternion(req.yaw,req.pitch,req.roll));
	}
	bool found = false;
	for(Frame &frame : frames)
	{
		if(f.name == frame.name)
		{
			found = true;
		}
	}
	if(!found)
	{
		f.dirty = true;
		frames.push_back(f);
		std::cout << "[tfPublisher]: Adding new frame: " << f.name << std::endl;
	}else
	{
		std::cout << "[tfPublisher]: Error, frame " << f.name << " is already there!" << std::endl;
	}
	res.success = !found;
	return true;
}
/*Adds some delta offset to the coordinate frame with respect to its parent frame.*/
bool addDeltaCB(detection_msgs::AddFrame::Request &req, detection_msgs::AddFrame::Response &res)
{
	Frame f(req.name);
	if(req.useXYZ)
	{
		f.transform.setOrigin(tf2::Vector3(req.x,req.y,req.z));
	}
	if(req.useRotation)
	{
		f.transform.setRotation(tf2::Quaternion(req.yaw,req.pitch,req.roll));
	}
	bool found = false;
	for(Frame &frame : frames)
	{
		if(f.name == frame.name)
		{
			frame.transform = frame.transform * f.transform;
			frame.dirty = true;
			found = true;
			std::cout << "[tfPublisher]: Adding delta transform to existing frame: " << f.name << std::endl;
		}
	}
	res.success = found;
	if(!found)
	{
		ROS_WARN("Frame %s not found!",f.name.c_str());
	}
	return true;
}
/*Sets the coordinate frame to the give pose*/
bool setFrameCB(detection_msgs::SetFrame::Request &req, detection_msgs::SetFrame::Response &res)
{
	Frame f(req.name);
	f.transform.setOrigin(tf2::Vector3(req.t.translation.x,req.t.translation.y,req.t.translation.z));
	f.transform.setRotation(tf2::Quaternion(req.t.rotation.x,req.t.rotation.y,req.t.rotation.z,req.t.rotation.w));
	bool found = false;
	for(Frame &frame : frames)
	{
		if(f.name == frame.name)
		{
			frame.transform = f.transform;
			frame.dirty = true;
			found = true;
			std::cout << "[tfPublisher]: Set transform for existing frame: " << f.name << std::endl;
		}
	}
	res.success = found;
	if(!found)
	{
		ROS_WARN("Frame %s not found!",f.name.c_str());
	}
	return true;
}
//TODO delete static transform broadcaster
bool deleteFrameCB(detection_msgs::AddFrame::Request &req, detection_msgs::AddFrame::Response &res)
{
	res.success = false;
	for(size_t i = 0; i < frames.size(); i++)
	{
		if(req.name == frames.at(i).name)
		{
			res.success = true;
			frames.erase(frames.begin()+i);
			std::cout << "[tfPublisher]: Deleting frame: " << req.name << std::endl;
			return true;
		}
	}
	return true;
}
/*Sends all available frames*/
bool getFramesCB(detection_msgs::GetFrames::Request &req, detection_msgs::GetFrames::Response &res)
{
	for(Frame f : frames)
	{
		res.names.push_back(f.name);
		geometry_msgs::Transform t;
		t.rotation.x = f.transform.getRotation().x();
		t.rotation.y = f.transform.getRotation().y();
		t.rotation.z = f.transform.getRotation().z();
		t.rotation.w = f.transform.getRotation().w();
		t.translation.x = f.transform.getOrigin().x();
		t.translation.y = f.transform.getOrigin().y();
		t.translation.z = f.transform.getOrigin().z();
		res.transforms.push_back(t);
	}
	return true;
}

bool saveCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	std::string package_path = ros::package::getPath("detection");
	ROS_INFO("Storing transforms under ’%s’.",(package_path+"/settings/frames.txt").c_str());
	return storeTransforms(frames,package_path+"/settings/frames.txt");
}

void publishKinectFrames(tf2_ros::StaticTransformBroadcaster& staticBroadcaster)
{
	tf2::Transform t(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));
	geometry_msgs::TransformStamped ts;
	ts = TfHelper::toTransformStamped(t,ros::Time::now(),"kinect2_jetson_link","kinect2_jetson_rgb_optical_frame");
	staticBroadcaster.sendTransform(ts);
	//This is normally published by the individual camera, but deactivated for better reliability
	tf2::Transform t2(tf2::Quaternion(-0.003,0.001,0.004,1.0),tf2::Vector3(-0.053,0.0,0.012));
	ts = TfHelper::toTransformStamped(t2,ros::Time::now(), "kinect2_jetson_rgb_optical_frame","kinect2_jetson_ir_optical_frame");
	staticBroadcaster.sendTransform(ts);

	tf2::Transform t3(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));
	ts = TfHelper::toTransformStamped(t3,ros::Time::now(),"kinect2_xavier_link", "kinect2_xavier_rgb_optical_frame");
	staticBroadcaster.sendTransform(ts);
	//This is normally published by the individual camera, but deactivated for better reliability
	tf2::Transform t4(tf2::Quaternion(-0.003,-0.002,0.001,1.0),tf2::Vector3(-0.052,0.0,0.003));
	ts = TfHelper::toTransformStamped(t4,ros::Time::now(),"kinect2_xavier_rgb_optical_frame", "kinect2_xavier_ir_optical_frame");
	staticBroadcaster.sendTransform(ts);
}

int main (int argc, char* argv[])
{
	//Init ros stuff
	ros::init (argc, argv, "tfPublisher");
	nh = boost::make_shared<ros::NodeHandle>();
	static tf2_ros::StaticTransformBroadcaster staticBroadcaster;
	tf2_ros::TransformListener tfListener(tfBuffer);

	std::string package_path = ros::package::getPath("detection");
	if(loadTransforms(frames,package_path+"/settings/frames.txt"))//Loads the last stored world transform
	{
		ROS_INFO("[tfPublisher]: Loaded last transforms!");
	}else{
		ROS_WARN("[tfPublisher]: No transforms loaded. Please add some using the tfPublisherClient.");

		frames.push_back(Frame("/world"));
	}

	ros::ServiceServer service1 = nh->advertiseService("/tfPublisher/addNewFrame",addNewFrameCB);
	ros::ServiceServer service2 = nh->advertiseService("/tfPublisher/addDelta",addDeltaCB);
	ros::ServiceServer service3 = nh->advertiseService("/tfPublisher/setFrame",setFrameCB);
	ros::ServiceServer service4 = nh->advertiseService("/tfPublisher/save",saveCB);
	ros::ServiceServer service5 = nh->advertiseService("/tfPublisher/getFrames",getFramesCB);
	ros::ServiceServer service6 = nh->advertiseService("/tfPublisher/deleteFrame",deleteFrameCB);
	//Redraws the current world frame in the pcl viewer.
	ros::Rate r(UPDATE_RATE);
	std::cout << "Start sleep!\n";
	ros::Time::sleepUntil(ros::Time(1));
	std::cout << "[tfPublisher]: Starting loop." << std::endl;

	publishKinectFrames(staticBroadcaster);

	while (ros::ok())
	{
		for(size_t i = 0; i < frames.size(); i++)//Publish all tf-frames
		{
			Frame f = frames.at(i);
			if(f.name.size() == 0) continue;

			if(i == 0) //Lets send the transformation from camera to world!
			{
				tf2::Transform onlyLoc(tf2::Quaternion(0,0,0,1),f.transform.getOrigin());
				tf2::Transform onlyRot(f.transform.getRotation(),tf2::Vector3(0,0,0));
				tf2::Transform t = onlyLoc * onlyRot; //First translation from camera to world position, then rotation into world rotation.
				geometry_msgs::TransformStamped ts;
				ts = TfHelper::toTransformStamped(t,ros::Time::now(),CLOUD_FRAME,f.name);
				if(f.dirty)
				{
					f.dirty = false;
					staticBroadcaster.sendTransform(ts);
				}
			}
			else //Lets send the transformation from world to other locations!
			{
				geometry_msgs::TransformStamped ts;
				ts = TfHelper::toTransformStamped(f.transform,ros::Time::now(),"/world",f.name);
				if(f.dirty)
				{
					f.dirty = false;
					staticBroadcaster.sendTransform(ts);
				}
			}
		}

		ros::spinOnce ();
		r.sleep();
	}
	std::cout << "[tfPublisher]: Quit." << std::endl;
	return 0;
}
