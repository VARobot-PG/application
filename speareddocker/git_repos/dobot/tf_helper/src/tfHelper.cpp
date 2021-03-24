/*
 * TfHelper.cpp
 *
 *  Created on: May 20, 2019
 *      Author: philipf
 */

#include "../include/tf_helper/tfHelper.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

TfHelper::TfHelper()
{
	listener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
}

bool TfHelper::transformVector(const std::string source_frame, const std::string target_frame, const ros::Time time, tf2::Vector3 &vec)
{
	tf2::Transform t(tf2::Quaternion(0,0,0,1),vec);
	if(transformTransform(source_frame,target_frame,time,t))
	{
		vec = t.getOrigin();
		return true;
	}
	return false;
}
bool TfHelper::transformQuaternion(const std::string source_frame, const std::string target_frame, const ros::Time time, tf2::Quaternion &rot)
{
	tf2::Transform t(rot,tf2::Vector3(0,0,0));
	if(transformTransform(source_frame,target_frame,time,t))
	{
		rot = t.getRotation();
		return true;
	}
	return false;
}
bool TfHelper::transformTransform(const std::string source_frame, const std::string target_frame, const ros::Time time, tf2::Transform &transform)
{
	tf2::Transform other;
	if(getTransform(source_frame, target_frame, time, other))
	{
		transform = other.inverse() * transform;
		return true;
	}
	return false;
}

bool TfHelper::getTransform(const std::string source_frame, const std::string target_frame, const ros::Time time, tf2::Transform &transform)
{
	tf2::Stamped<tf2::Transform> t;
	try{
		geometry_msgs::TransformStamped tS;
		tS = tfBuffer.lookupTransform(source_frame,target_frame,time);
		tf2::convert(tS,t);
		transform = t;
	}catch(tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
		return false;
	}
	return true;
}


