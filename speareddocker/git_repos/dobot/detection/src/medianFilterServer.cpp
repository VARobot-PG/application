/*
 * medianFilterServer.cpp
 *
 *	This runs on the jetson and xavier, and offers the service to calculate a noise reduced pointcloud. This takes up to 30 seconds!
 *	The benefit is, all the samples can be processed directly on the server and do not have to be send to the client. Only the result is send.
 *	The client gets notified when the new pointcloud is ready.
 *  Created on: Aug 14, 2019
 *      Author: philipf
 */
#include <stdlib.h>
#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
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

#include <detection_msgs/SetPointcloud.h>
#include <detection_msgs/SetString.h>
#include <std_srvs/Empty.h>

ros::Subscriber sub;
tf2_ros::Buffer tfBuffer;
boost::shared_ptr<ros::NodeHandle> nh;

const size_t MAX_NUM_SAMPLES = 24;
cloud::MedianFilter medianFilter(false);
bool cloudFinished = true;
std::queue<std::string> pointcloudSubscribers;

std::string cameraName = "kinect2_jetson";

/**
 * Callback if new pointcloud has been recorded.
 * @param pointcloud The pointcloud will be only applied on startup, or after y button has been pressed.
 */
void pointcloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
	if(cloudFinished) return;
	if(pointcloudSubscribers.size() == 0) return;
	if(msg->width == 0) return;

	pcl::PointCloud<cloud::PointType>::Ptr sample = boost::make_shared<pcl::PointCloud<cloud::PointType> >();

	pcl::fromROSMsg(*msg,*sample);

	medianFilter.push_back(sample);
	std::cout << "[medianFilterServer]: Recorded " << medianFilter.size() << " of " << MAX_NUM_SAMPLES << std::endl;
	if(medianFilter.size() >= MAX_NUM_SAMPLES)
	{
		cloudFinished = true;
		sub.shutdown();
	}
}

/*Registers a node as a pointcloud subscriber. After enough clouds have been received and processed, the pointcloud subscriber will be notified.*/
bool askPointcloudCB(detection_msgs::SetString::Request &req, detection_msgs::SetString::Response &res)
{
	pointcloudSubscribers.push(req.msg);
	ROS_INFO("%i clients in pointcloud queue.",(int)pointcloudSubscribers.size());
	if(pointcloudSubscribers.size() == 1)
	{
		cloudFinished = false;
		medianFilter.clear();
		sub = nh->subscribe<sensor_msgs::PointCloud2> ("/" + cameraName + "/hd/points", 2, pointcloudCallback);
		ROS_INFO("Subscribing to %s!",sub.getTopic().c_str());
	}
	return true;
}

int main (int argc, char* argv[])
{
	if(argc == 1)
	{
		std::cout << "Usage: rosrun detection medianFilterServer CAMERA_NAME\n";
		return -1;
	}
	cameraName = argv[1];

	std::cout << "CameraName = " << cameraName << std::endl;

	ros::init (argc, argv, cameraName + "_MedianFilter");
	nh = boost::make_shared<ros::NodeHandle>();
	tf2_ros::TransformListener tfListener(tfBuffer);

	//Check if publisher exists!
	sub = nh->subscribe<sensor_msgs::PointCloud2> ("/" + cameraName + "/hd/points", 2, pointcloudCallback);
	ros::Duration(1.0).sleep();
	ros::spinOnce();
	if(sub.getNumPublishers() == 0)
	{
		ROS_WARN("No publisher for /%s/hd/points",cameraName.c_str());
		sub.shutdown();
	}else{
		ROS_INFO("Succesfully found publisher!");
		sub.shutdown();
	}

	ros::ServiceServer service7 = nh->advertiseService("/" + cameraName + "/askForPointcloud",askPointcloudCB);

	ros::Rate r(4);
	while(ros::ok())
	{
		if(cloudFinished)//Is the sampling done?
		{
			pcl::PointCloud<cloud::PointType>::Ptr cloud = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
			ROS_INFO("Starting median filter!");
			medianFilter.compute(cloud);
			ROS_INFO("Sending answers!");
			detection_msgs::SetPointcloud srv;
			pcl::PCLPointCloud2 cloud2;
			pcl::toPCLPointCloud2(*cloud,cloud2);
			pcl_conversions::fromPCL(cloud2,srv.request.cloud);
			medianFilter.clear();
			while(pointcloudSubscribers.size() > 0)//Send the noise reduced pointcloud to all clients
			{
				std::string topic = pointcloudSubscribers.front();
				pointcloudSubscribers.pop();
				ros::ServiceClient c = nh->serviceClient<detection_msgs::SetPointcloud>(topic);
				if(!c.call(srv))
				{
					ROS_ERROR("Service failed! Have you added the other machine to my '/etc/hosts' file?");
				}
				ROS_INFO("Send pointcloud reply to %s",topic.c_str());
			}
			cloudFinished = false;
		}
		ros::spinOnce();
		r.sleep();
	}
}
