/*
 * gridDetection.cpp
 *
 * Subscribes to the kinect's PointCloud and detects the construction grid.
 *
 *  Created on: 1 August, 2019
 *      Author: Sven Peeters
 */

#include <iostream>
#include <string>
#include <memory>
#include <cmath>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <detection_msgs/DetectedGrid.h>

#include "../include/cloud.h"
#include <marker_helper/markerHelper.h>
#include <std_msgs/Float64.h>

namespace State
{
	enum State
	{
		INIT,RECORDING,PROCESSING,DONE
	};
}

std::string TARGET_FRAME = "world";

tf2::Vector3 startBox = tf2::Vector3(0.0,0.0,0.005);
tf2::Vector3 endBox = tf2::Vector3(0.18,0.4,0.15);

double thresholdGreen = 100;
double thresholdPink = 100;


tf2::Vector3 baseColor = tf2::Vector3(160 ,190,120); //palm leaf
tf2::Vector3 offsetColor = tf2::Vector3(190,50,130); //smitten

tf2_ros::Buffer tfBuffer;

State::State currentState = State::INIT;
std::unique_ptr<MarkerHelper> markerManager;
std::shared_ptr<ros::NodeHandle> nh;
pcl::PointCloud<cloud::PointType>::Ptr currentScene;
ros::Time sceneTime;

detection_msgs::DetectedGrid detectedGrid;

/**
 * Callback for the subscriber that subscribes the point cloud from the kinect camera
 * @param pointcloud
 */
void pointcloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(currentState != State::RECORDING) return;
    if(msg->width == 0) return;

    sensor_msgs::PointCloud2 other;
    geometry_msgs::TransformStamped tS;
    try{
        tS = tfBuffer.lookupTransform(TARGET_FRAME,msg->header.frame_id,ros::Time(0));
        tf2::doTransform(*msg,other,tS);
        sceneTime = msg->header.stamp;
        pcl::fromROSMsg(other,*currentScene);
        currentState = State::PROCESSING;
    }catch(tf2::TransformException &ex)
    {
        ROS_INFO("UFF");
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
}





/**
 * This program detects the construction grid based on the pink and green points sticked to it. It creates a new frame
 * for the construction grid and publishes it.
 * PARAMS: rosrun detection gridDetection [NodeName] [Targetf2rame]
 */
int main (int argc, char** argv)
{
	std::string applicationName = "gridDetection";
	if(argc > 1) applicationName = argv[1];
	else std::cout << "Usage: rosrun detection gridDetection [NodeName] [Targetframe]\n";
	if(argc > 2) TARGET_FRAME = argv[2];

	ros::init (argc, argv, "~");
	nh = std::make_shared<ros::NodeHandle>();

	nh->param("/" + applicationName + "/startX",startBox.x());
	nh->param("/" + applicationName + "/startY",startBox.y());
	nh->param("/" + applicationName + "/startZ",startBox.z());
	nh->param("/" + applicationName + "/endX",endBox.x());
	nh->param("/" + applicationName + "/endY",endBox.y());
	nh->param("/" + applicationName + "/endZ",endBox.z());
    nh->param("/" + applicationName + "/thresholdGreen",thresholdGreen);
    nh->param("/" + applicationName + "/thresholdGreen",thresholdPink);

	ros::Subscriber sub;

	currentScene = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
    tf2_ros::TransformListener tfListener(tfBuffer);
	markerManager = std::make_unique<MarkerHelper>(nh,"/" + applicationName + "/marker","objects");

    sub = nh->subscribe<sensor_msgs::PointCloud2> ("/kinect2_xavier/hd/points", 1, pointcloudCallback);
	ros::Publisher anglePub = nh -> advertise<std_msgs::Float64>("/" + applicationName + "/gridAngleTo"+ TARGET_FRAME, 10);
	ros::Publisher gridPub = nh -> advertise<detection_msgs::DetectedGrid>("/"+applicationName + "/detectedGrid",10);
	ros::Publisher greenDebugCloudPub = nh->advertise<sensor_msgs::PointCloud2>("/" + applicationName + "/greenDebugCloud",2);
    ros::Publisher pinkDebugCloudPub = nh->advertise<sensor_msgs::PointCloud2>("/" + applicationName + "/pinkDebugCloud",2);

	tf2_ros::StaticTransformBroadcaster br;
	tf2::Transform transform;

	baseColor = cloud::convertToLabColorspace(pcl::PointXYZRGB(baseColor.x(), baseColor.y(), baseColor.z()));
    offsetColor = cloud::convertToLabColorspace(pcl::PointXYZRGB(offsetColor.x(), offsetColor.y(), offsetColor.z()));

    ros::Rate r(8);
	std::cout << "Starting loop!\n";
	while(ros::ok())
	{
		switch(currentState)
		{
			case State::INIT:
			{
				currentState = State::RECORDING;
				std::cout << "Done Init!\n";
				break;
			}
			case State::PROCESSING:
			{
				double x1,y1,z1,x2,y2,z2, tg, tp;
				nh->getParam("/" + applicationName + "/startX",x1);
				nh->getParam("/" + applicationName + "/startY",y1);
				nh->getParam("/" + applicationName + "/startZ",z1);
				nh->getParam("/" + applicationName + "/endX",x2);
				nh->getParam("/" + applicationName + "/endY",y2);
				nh->getParam("/" + applicationName + "/endZ",z2);
                nh->getParam("/" + applicationName + "/thresholdGreen",tg);
                nh->getParam("/" + applicationName + "/thresholdPink",tp);
				startBox = tf2::Vector3(x1,y1,z1);
				endBox = tf2::Vector3(x2,y2,z2);

                // publish bounding box

                ROS_INFO_STREAM("cutting cloud");
				//Using a box cut to only keep points inside the pickup area.
				pcl::PointCloud<cloud::PointType>::Ptr cutScene = boost::make_shared<pcl::PointCloud<cloud::PointType>>();

				tf2::Vector3 startBox_tf = tf2::Vector3(startBox.x(), startBox.y(), startBox.z());
                tf2::Vector3 endBox_tf = tf2::Vector3(endBox.x(), endBox.y(), endBox.z());
				cloud::cutPointCloudRect(currentScene,cutScene, startBox_tf, endBox_tf);

                pcl::PointCloud<cloud::PointType>::Ptr pinkCloud = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
                pcl::PointCloud<cloud::PointType>::Ptr greenCloud = boost::make_shared<pcl::PointCloud<cloud::PointType>>();

                // publish bounding box
                markerManager->publishBox(startBox_tf,endBox_tf,"world",0,0,1.0,0,0.33,"BoundingBox",0);



                ROS_INFO_STREAM("separating by color");
                ROS_INFO_STREAM("cutScene size: " << cutScene->size());

                for(size_t i = 0; i < cutScene->size(); i++)
                {
                    //tf2::Vector3 pointColor = tf2::Vector3(cutScene->at(i).r,cutScene->at(i).g,cutScene->at(i).b);
                    tf2::Vector3 pointColor = cloud::convertToLabColorspace(cutScene->at(i));
                    double dist_pink = pointColor.distance(offsetColor);
                    if(dist_pink < tp)
                        pinkCloud->push_back(cutScene->at(i));

                    double dist_green = pointColor.distance(baseColor);
                    if(dist_green < tg)
                        greenCloud->push_back(cutScene->at(i));
                }
                ROS_INFO_STREAM("threshold green: " << tg);
                ROS_INFO_STREAM("threshold pink: " << tp);
                ROS_INFO_STREAM("pink cloud empty: " << pinkCloud->empty());
                ROS_INFO_STREAM("green cloud empty: " << greenCloud->empty());

                pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;

                // filter outliers out
                outrem.setInputCloud(greenCloud);
                pcl::PointCloud<cloud::PointType>::Ptr greenCloud_filtered = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
                outrem.setRadiusSearch(0.015);
                outrem.setMinNeighborsInRadius(100);
                outrem.filter(*greenCloud_filtered);

                outrem.setInputCloud(pinkCloud);
                pcl::PointCloud<cloud::PointType>::Ptr pinkCloud_filtered = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
                outrem.setRadiusSearch(0.015);
                outrem.setMinNeighborsInRadius(100);
                outrem.filter(*pinkCloud_filtered);

                if(!(pinkCloud_filtered->empty() || greenCloud_filtered->empty())){

                    std::vector<pcl::PointCloud<cloud::PointType>::Ptr> segments_base;
                    std::vector<pcl::PointCloud<cloud::PointType>::Ptr> segments_offset;

                    ROS_INFO_STREAM("calculating centroids");

                    pcl::CentroidPoint<pcl::PointXYZRGB> centroid_base;
                    pcl::CentroidPoint<pcl::PointXYZRGB> centroid_offset;
                    // add each point of the segment to the centroid

                    for(auto point : greenCloud_filtered->points){
                        centroid_base.add(point);
                    }
                    for(auto point : pinkCloud_filtered->points){
                        centroid_offset.add(point);
                    }

                    //calculate the middle of both centroids

                    pcl::PointXYZ center_base(0,0,0);
                    centroid_base.get(center_base);

                    pcl::PointXYZ center_offset(0,0,0);
                    centroid_offset.get(center_offset);

                    ROS_INFO_STREAM("Base centroid at: " << center_base.x << "," << center_base.y << "," << center_base.z);
                    ROS_INFO_STREAM("Offset centroid at: " << center_offset.x << "," << center_offset.y << "," << center_offset.z);

                    // calculate origin and rotation of new frame
                    tf2::Vector3 origin(center_base.x, center_base.y, center_base.z);
                    tf2::Vector3 offset(center_offset.x,center_offset.y,center_offset.z);
                    tf2::Vector3 axis =  offset - origin;

                    tf2::Vector3 base(1,0, origin.z());
                    axis.setZ(origin.z());

                    ROS_INFO_STREAM("Axis vecotr: " << axis.x() << "," << axis.y() << "," << axis.z());

                    auto angle = axis.angle(base);

                    tfScalar conv_angle;
                    tf2::Quaternion rot;


                    if(axis.x() < 0 && axis.y() > 0)
                        conv_angle = angle;
                    else if(axis.x() < 0 && axis.y() < 0)
                        conv_angle = -angle;
                    else if(axis.x() > 0 && axis.y() < 0)
                        conv_angle = -angle;
                    else
                        conv_angle = angle;

                    rot.setRPY(0,0, conv_angle);
                    // publish the rotation

                    std_msgs::Float64 angleMessage;

                    angleMessage.data = conv_angle * 180 / M_PI;

                    anglePub.publish(angleMessage);


                    ROS_INFO_STREAM("Frame origin at: " << origin.x() << "," << origin.y() << "," << origin.z());
                    ROS_INFO_STREAM("Frame rotation: " << rot.x() << "," << rot.y() << "," << rot.z() << "," << rot.w());

                    // publish transform
                    geometry_msgs::TransformStamped transformStamped;

                    transformStamped.header.stamp = ros::Time::now();
                    transformStamped.header.frame_id = "world";
                    transformStamped.child_frame_id = "gridDetection";
                    transformStamped.transform.translation.x = origin.x();
                    transformStamped.transform.translation.y = origin.y();
                    transformStamped.transform.translation.z = origin.z();
                    transformStamped.transform.rotation.x = rot.x();
                    transformStamped.transform.rotation.y = rot.y();
                    transformStamped.transform.rotation.z = rot.z();
                    transformStamped.transform.rotation.w = rot.w();

                    br.sendTransform(transformStamped);


                    // publish marker for both centroids
                    geometry_msgs::Point cb_point;
                    cb_point.x = origin.x();
                    cb_point.y = origin.y();
                    cb_point.z = origin.z();
                    markerManager->publishMarker(cb_point,TARGET_FRAME,0,1,1,0,"greenCentroid",1.0);

                    geometry_msgs::Point co_point;
                    co_point.x = center_offset.x;
                    co_point.y = center_offset.y;
                    co_point.z = center_offset.z;
                    markerManager->publishMarker(co_point,TARGET_FRAME,0,1,1,0,"pinkCentroid",1.0);

                    // publish pink and green cloud for debugging
                    if(greenDebugCloudPub.getNumSubscribers() > 0)
                    {
                        sensor_msgs::PointCloud2 debugCloudOut;
                        pcl::toROSMsg(*greenCloud_filtered, debugCloudOut);
                        debugCloudOut.header.stamp = ros::Time::now();
                        debugCloudOut.header.frame_id = TARGET_FRAME;
                        greenDebugCloudPub.publish(debugCloudOut);
                    }
                    if(pinkDebugCloudPub.getNumSubscribers() > 0)
                    {
                        sensor_msgs::PointCloud2 debugCloudOut;
                        pcl::toROSMsg(*pinkCloud_filtered, debugCloudOut);
                        debugCloudOut.header.stamp = ros::Time::now();
                        debugCloudOut.header.frame_id = TARGET_FRAME;
                        pinkDebugCloudPub.publish(debugCloudOut);
                    }
                }


				currentState = State::DONE;
				break;
			}
			case State::DONE:
			{
				currentState = State::RECORDING;
				break;
			}
			default:
			{

			}
		}
		ros::spinOnce();
		r.sleep();
	}


}
