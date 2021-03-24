/*
 * dominoDetection.cpp
 *
 * Subscribes to the kinect's PointCloud and detects domino blocks.
 *
 *  Created on: 30 September, 2019
 *      Author: philipf
 */

#include <iostream>
#include <fstream>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <ros/time.h>
#include <eigen3/Eigen/Geometry>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <detection_msgs/DetectedDomino.h>
#include <detection_msgs/DetectedDominos.h>

#include "../include/cloud.h"
#include "../include/medianFilter.h"
#include <marker_helper/markerHelper.h>

namespace State
{
	enum State
	{
		INIT,RECORDING,PROCESSING,DONE
	};
}

std::string TARGET_FRAME = "world";

//Only points inside the box are relevant
tf2::Vector3 startBox = tf2::Vector3(0.0,-1.35,-0.1);
tf2::Vector3 endBox = tf2::Vector3(0.4,-0.9,0.4);

//What is the min and max size of points inside a graspable cluster?
int minClusterSize = 50;
int maxClusterSize = 800;
//All non graspable points which are closer than this to a graspable surface will be merged with that surface
double distanceThreshold = 0.1;

std::string cameraTopic = "kinect2_xavier/qhd/points";

//Used to detect the background, which can be cut away
double backgroundFilterThreshold = 120.0;
int backgroundR = 170;
int backgroundG = 170;
int backgroundB = 170;
bool cutAwayBackbround = true;

State::State currentState = State::INIT;
tf2_ros::Buffer tfBuffer;
std::unique_ptr<MarkerHelper> markerManager;
std::shared_ptr<ros::NodeHandle> nh;
pcl::PointCloud<cloud::PointType>::Ptr currentScene;
ros::Time sceneTime;
/*Called if kinect camera has a new pointcloud*/
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
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
		return;
	}
}
/*Gets something similar to the distance between two points. Quicker in computation than the eucledian distance.*/
double getSimpleDistance(tf2::Vector3 a, tf2::Vector3 b)
{
	return std::fabs(a.x() - b.x()) + std::fabs(a.y() - b.y()) + std::fabs(a.z() - b.z());
}

/*Finds four points (corners) which have the highest distance to each other. Based on them, the rotation is detected.*/
bool getBlockRotation(pcl::PointCloud<cloud::PointType>::Ptr scene, tf2::Quaternion& rot, std::vector<tf2::Vector3>& borderPoints)
{
	if(scene->size() <= 3 ) return false;

	tf2::Vector3 aY2d = tf2::Vector3(0,0,0);
	tf2::Vector3 bY2d = tf2::Vector3(0,0,0);
	tf2::Vector3 aY3d = tf2::Vector3(0,0,0);
	tf2::Vector3 bY3d = tf2::Vector3(0,0,0);
	double maxDistance = -1.0;
	for(size_t i = 0; i < scene->size(); i++)//Finds two border points
	{
		for(size_t j = 0; j < scene->size(); j++)
		{
			if(i == j) continue;
			tf2::Vector3 a(scene->at(i).x,scene->at(i).y,0);
			tf2::Vector3 b(scene->at(j).x,scene->at(j).y,0);
			double d = a.distance(b);
			if(d > maxDistance || maxDistance < 0)
			{
				aY2d = a; bY2d = b;
				aY3d = tf2::Vector3(scene->at(i).x,scene->at(i).y,scene->at(i).z);
				bY3d = tf2::Vector3(scene->at(j).x,scene->at(j).y,scene->at(j).z);
				maxDistance = d;
			}
		}
	}

	tf2::Vector3 aR2d = tf2::Vector3(0,0,0);
	tf2::Vector3 bR2d = tf2::Vector3(0,0,0);
	tf2::Vector3 aR3d = tf2::Vector3(0,0,0);
	tf2::Vector3 bR3d = tf2::Vector3(0,0,0);
	maxDistance = -1.0;
	for(size_t i = 0; i < scene->size(); i++)//Finds the other two border points
	{
		for(size_t j = 0; j < scene->size(); j++)
		{
			if(i == j) continue;
			tf2::Vector3 a(scene->at(i).x,scene->at(i).y,0);
			tf2::Vector3 b(scene->at(j).x,scene->at(j).y,0);
			double d1 = a.distance(aY2d);
			double d2 = a.distance(bY2d);
			double d = a.distance(b) + d1 + d2 + b.distance(aY2d) + b.distance(bY2d);
			if(d > maxDistance || maxDistance < 0)
			{
				if(d1 < d2)
				{
					aR2d = a;
					bR2d = b;
					aR3d = tf2::Vector3(scene->at(i).x,scene->at(i).y,scene->at(i).z);
					bR3d = tf2::Vector3(scene->at(j).x,scene->at(j).y,scene->at(j).z);
				}else{
					bR2d = a;
					aR2d = b;
					bR3d = tf2::Vector3(scene->at(i).x,scene->at(i).y,scene->at(i).z);
					aR3d = tf2::Vector3(scene->at(j).x,scene->at(j).y,scene->at(j).z);
				}
				maxDistance = d;
			}
		}
	}

	//Find two vectors, which are parallel to each other. These point in the block direction.
	tf2::Vector3 normalEvenA = aY2d - bR2d;
	normalEvenA.normalize();
	tf2::Vector3 normalEvenB = aR2d - bY2d;
	normalEvenB.normalize();

	tf2::Vector3 forwardVector;
	forwardVector = (normalEvenA + normalEvenB) / 2.0;
	forwardVector.normalize();

	//This calculates the angle between two vectors.
	tf2::Vector3 forwardVectorGlobal(1,0,0);
	tf2::Vector3 rotAxis = forwardVectorGlobal.cross(forwardVector);
	rot.setX(rotAxis.x());
	rot.setY(rotAxis.y());
	rot.setZ(rotAxis.z());
	rot.setW(std::sqrt(forwardVector.length2() * forwardVectorGlobal.length2()) + forwardVector.dot(forwardVectorGlobal));
	rot.normalize();

	borderPoints.push_back(aY3d);
	borderPoints.push_back(aR3d);
	borderPoints.push_back(bY3d);
	borderPoints.push_back(bR3d);
	return true;
}
/*Gets the color of both sides of the domino block, based on the corner points.*/
bool getDominoColors(std::vector<tf2::Vector3> boundaryPoints, pcl::PointCloud<cloud::PointType>::Ptr cloud, tf2::Vector3 &color1, tf2::Vector3 &color2)
{
	if(boundaryPoints.size() < 4) return false;

	color1 = tf2::Vector3(0,0,0);
	tf2::Vector3 color1Squared(0,0,0);
	size_t counter1 = 0;
	color2 = tf2::Vector3(0,0,0);
	tf2::Vector3 color2Squared(0,0,0);
	size_t counter2 = 0;

	std::vector<bool> isA;
	//For each point, get the side it is laying on and calculate the average color
	for(size_t i = 0; i < cloud->size(); i++)
	{
		tf2::Vector3 p(cloud->at(i).x,cloud->at(i).y,cloud->at(i).z);
		double d1 = p.distance(boundaryPoints.at(0)) + p.distance(boundaryPoints.at(1));
		double d2 = p.distance(boundaryPoints.at(2)) + p.distance(boundaryPoints.at(3));
		if(d1 < d2)
		{
			color1 += tf2::Vector3(cloud->at(i).r,cloud->at(i).g,cloud->at(i).b);
			color1Squared += tf2::Vector3(cloud->at(i).r,cloud->at(i).g,cloud->at(i).b) * tf2::Vector3(cloud->at(i).r,cloud->at(i).g,cloud->at(i).b);
			isA.push_back(true);
			counter1++;
		}else{
			isA.push_back(false);
			color2 += tf2::Vector3(cloud->at(i).r,cloud->at(i).g,cloud->at(i).b);
			color2Squared += tf2::Vector3(cloud->at(i).r,cloud->at(i).g,cloud->at(i).b) * tf2::Vector3(cloud->at(i).r,cloud->at(i).g,cloud->at(i).b);
			counter2++;
		}
	}
	//Average
	color1 /= (double)counter1;
	color2 /= (double)counter2;
	//Average squared
	color1Squared /= (double)counter1;
	color2Squared /= (double)counter2;

	tf2::Vector3 variance1 = color1Squared - color1;
	tf2::Vector3 variance2 = color2Squared - color2;

	counter1 = 0;
	counter2 = 0;

	tf2::Vector3 averageColor1(0,0,0);
	tf2::Vector3 averageColor2(0,0,0);
	for(size_t i = 0; i < cloud->size(); i++)
	{
		tf2::Vector3 color = tf2::Vector3(cloud->at(i).r,cloud->at(i).g,cloud->at(i).b);
		if(isA.at(i))
		{
			if(color.x() < color1.x() < variance1.x() && color.y() < color1.y() < variance1.y() && color.z() < color1.z() < variance1.z())
			{
				averageColor1 += color;
				counter1++;
			}else{
				cloud->at(i).r = 0;
				cloud->at(i).g = 0;
				cloud->at(i).b = 0;
			}
		}else{
			if(color.x() < color2.x() < variance2.x() && color.y() < color2.y() < variance2.y() && color.z() < color2.z() < variance2.z())
			{
				averageColor2 += color;
				counter2++;
			}else{
				cloud->at(i).r = 0;
				cloud->at(i).g = 0;
				cloud->at(i).b = 0;
			}
		}
	}

	color1 = averageColor1 / counter1;
	color2 = averageColor2 / counter2;

	return true;
}

int main (int argc, char** argv)
{
	std::string applicationName = "dominoDetection";
	if(argc > 1) applicationName = argv[1];
	else std::cout << "Usage: rosrun detection dominoDetection [NodeName=dominoDetection] [TargetFrame=world]\n";
	if(argc > 2) TARGET_FRAME = argv[2];

	ros::init (argc, argv, applicationName);
	nh = std::make_shared<ros::NodeHandle>();

	ros::Publisher objectsPub = nh->advertise<detection_msgs::DetectedDominos>("/" + applicationName + "/dominos",10);
	ros::Publisher debugClusterPub = nh->advertise<sensor_msgs::PointCloud2>("/" + applicationName + "/debugCluster",2);
	ros::Publisher debugGraspPub = nh->advertise<sensor_msgs::PointCloud2>("/" + applicationName + "/debugGrasp",2);

	ros::Duration(0.5).sleep();

	//These parameters can be changed during runtime
	nh->setParam("/" + applicationName + "/startX",startBox.x());
	nh->setParam("/" + applicationName + "/startY",startBox.y());
	nh->setParam("/" + applicationName + "/startZ",startBox.z());
	nh->setParam("/" + applicationName + "/endX",endBox.x());
	nh->setParam("/" + applicationName + "/endY",endBox.y());
	nh->setParam("/" + applicationName + "/endZ",endBox.z());
	nh->param("/" + applicationName + "/minClusterSize",minClusterSize);
	nh->param("/" + applicationName + "/maxClusterSize",maxClusterSize);
	nh->param("/" + applicationName + "/distanceThreshold",distanceThreshold);

	nh->param("/" + applicationName + "/backgroundColor/r",backgroundR);
	nh->param("/" + applicationName + "/backgroundColor/g",backgroundG);
	nh->param("/" + applicationName + "/backgroundColor/b",backgroundB);
	nh->param("/" + applicationName + "/backgroundFilterThreshold",backgroundFilterThreshold);
	nh->param("/" + applicationName + "/cutAwayBackbround",cutAwayBackbround);
	nh->param("/" + applicationName + "/cameraTopic",cameraTopic);

	currentScene = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
	tf2_ros::TransformListener tfListener(tfBuffer);
	markerManager = std::make_unique<MarkerHelper>(nh,"/" + applicationName + "/marker","objects");
	ros::Subscriber sub = nh->subscribe<sensor_msgs::PointCloud2> (cameraTopic, 1, pointcloudCallback);

	ros::Rate r(4);
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
				//Update all parameters
				double x1,y1,z1,x2,y2,z2;
				nh->getParam("/" + applicationName + "/startX",x1);
				nh->getParam("/" + applicationName + "/startY",y1);
				nh->getParam("/" + applicationName + "/startZ",z1);
				nh->getParam("/" + applicationName + "/endX",x2);
				nh->getParam("/" + applicationName + "/endY",y2);
				nh->getParam("/" + applicationName + "/endZ",z2);
				startBox = tf2::Vector3(x1,y1,z1);
				endBox = tf2::Vector3(x2,y2,z2);

				nh->getParam("/" + applicationName + "/minClusterSize",minClusterSize);
				nh->getParam("/" + applicationName + "/maxClusterSize",maxClusterSize);
				nh->getParam("/" + applicationName + "/distanceThreshold",distanceThreshold);

				nh->getParam("/" + applicationName + "/backgroundColor/r",backgroundR);
				nh->getParam("/" + applicationName + "/backgroundColor/g",backgroundG);
				nh->getParam("/" + applicationName + "/backgroundColor/b",backgroundB);
				nh->getParam("/" + applicationName + "/backgroundFilterThreshold",backgroundFilterThreshold);
				nh->getParam("/" + applicationName + "/cutAwayBackbround",cutAwayBackbround);
				markerManager->clearAll();
				markerManager->publishBox(startBox,endBox,TARGET_FRAME,0,0,1.0,0,0.33,"BoundingBox",0);

				//Only keep points inside the box of interest.
				pcl::PointCloud<cloud::PointType>::Ptr cutScene = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
				cloud::cutPointCloudRect(currentScene,cutScene,startBox,endBox);

				//Cut away the background points
				pcl::PointCloud<cloud::PointType>::Ptr cutSceneRGB = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
				cloud::cutBackgroundColor(cutScene,cutSceneRGB,backgroundFilterThreshold,tf2::Vector3((double)backgroundR,(double)backgroundG,(double)backgroundB));
				std::cout << "Input points: " << cutSceneRGB->size() << std::endl;

				//Voxel grid sampling to reduce the number of points
				pcl::PointCloud<cloud::PointType>::Ptr cutSceneVox = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
				pcl::VoxelGrid<cloud::PointType> sor;
				sor.setInputCloud(cutSceneRGB);
				sor.setLeafSize(0.005f,0.005f,0.005f);
				sor.filter(*cutSceneVox);

				//If anybody is subscribed, publish this
				if(debugGraspPub.getNumSubscribers() > 0)
				{
					sensor_msgs::PointCloud2 debugCloudOut;
					pcl::toROSMsg(*cutSceneVox, debugCloudOut); //colorCloud
					debugCloudOut.header.stamp = ros::Time::now();
					debugCloudOut.header.frame_id = TARGET_FRAME;
					debugGraspPub.publish(debugCloudOut);
				}

				//Find the domino blocks by clustering close nearby points to one cluster
				pcl::PointCloud<cloud::PointType>::Ptr colorCloud = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
				std::vector<pcl::PointCloud<cloud::PointType>::Ptr> clouds;
				cloud::regionGrowingNew(cutSceneVox,colorCloud,clouds,minClusterSize,maxClusterSize,distanceThreshold);
				std::cout << "Found " << clouds.size() << " clusters!\n";
				//cloud::colorBasedRegionGrowing(cutSceneVox,colorCloud,clouds,minClusterSize,maxClusterSize,distanceThreshold,INT_MAX,INT_MAX);
				//If anybody is subscribed, publish this
				if(debugClusterPub.getNumSubscribers() > 0)
				{
					sensor_msgs::PointCloud2 debugCloudOut;
					pcl::toROSMsg(*colorCloud, debugCloudOut); //colorCloud
					debugCloudOut.header.stamp = ros::Time::now();
					debugCloudOut.header.frame_id = TARGET_FRAME;
					debugClusterPub.publish(debugCloudOut);
				}

				int counter = 0;
				std::vector<detection_msgs::DetectedDomino> dominos;
				//Find the median point for all graspable surfaces
				#pragma omp parallel for shared(dominos)
				for(size_t i = 0; i < clouds.size(); i++)
				{
					//Get the border points of this cluster. They represent the 4 corners
					pcl::PointCloud<cloud::PointType>::Ptr c = clouds.at(i);
					tf2::Quaternion partRotation = tf2::Quaternion(0,0,0,1);
					std::vector<tf2::Vector3> borderPoints;
					if(!getBlockRotation(clouds.at(i),partRotation,borderPoints))
					{
						continue;
					}
					//Check NaNs
					if(partRotation.x() != partRotation.x() || partRotation.y() != partRotation.y() ||
							partRotation.z() != partRotation.z() || partRotation.w() != partRotation.w())
					{
						continue;
					}

					//Get the color
					tf2::Vector3 colorA,colorB;
					getDominoColors(borderPoints,c,colorA,colorB);
					colorA /= 255.0;
					colorB /= 255.0;

					//Get the grasp point
					size_t medianPointId = 0;
					cloud::getMedianPoint(c,medianPointId);
					tf2::Vector3 medianPoint(c->at(medianPointId).x,c->at(medianPointId).y,c->at(medianPointId).z);

					//Visualization stuff:
					markerManager->publishArrow(tf2::Transform(partRotation,medianPoint + tf2::Vector3(0,0,0.02)),
																		TARGET_FRAME,i,0.05,1.0,1.0,1.0,"rotation");
					tf2::Vector3 centerOffset = (borderPoints.at(3)-borderPoints.at(0))/4.0;
					double width = (borderPoints.at(1)-borderPoints.at(0)).length();
					double length = centerOffset.length();
					markerManager->publishBox(tf2::Transform(partRotation,medianPoint - centerOffset),"world",tf2::Vector3(length*2.0,width,medianPoint.z()),
							i*2,colorA.x(),colorA.y(),colorA.z(),"block");

					width = (borderPoints.at(3)-borderPoints.at(2)).length();
					markerManager->publishBox(tf2::Transform(partRotation,medianPoint + centerOffset),"world",tf2::Vector3(length*2.0,width,medianPoint.z()),
							i*2+1,colorB.x(),colorB.y(),colorB.z(),"block");

					//Create the message and send it
					detection_msgs::DetectedDomino domino;
					domino.rotation = tf2::toMsg(partRotation);
					domino.graspPoint.x = c->at(medianPointId).x;
					domino.graspPoint.y = c->at(medianPointId).y;
					domino.graspPoint.z = c->at(medianPointId).z;
					markerManager->publishMarker(domino.graspPoint,TARGET_FRAME,i,255,255,255,"graspPoint");

					domino.colorFirst.resize(3);
					domino.colorFirst.at(0) = uint8_t(colorB.x()*255);
					domino.colorFirst.at(1) = uint8_t(colorB.y()*255);
					domino.colorFirst.at(2) = uint8_t(colorB.z()*255);

					domino.colorSecond.resize(3);
					domino.colorSecond.at(0) = uint8_t(colorA.x()*255);
					domino.colorSecond.at(1) = uint8_t(colorA.y()*255);
					domino.colorSecond.at(2) = uint8_t(colorA.z()*255);
					domino.timestamp = sceneTime;

					dominos.push_back(domino);
				}

				std::cout << "Sending messages!\n";
				detection_msgs::DetectedDominos msg;
				msg.dominos = dominos;
				msg.frame_id = TARGET_FRAME;
				objectsPub.publish(msg);

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
