/*
 * graspPointDetection.cpp
 *
 * Subscribes to the kinect's PointCloud and detects graspable surfaces.
 *
 *  Created on: 22 April, 2019
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

#include <detection_msgs/DetectedObject.h>
#include <detection_msgs/DetectedObjects.h>

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

std::string TARGET_FRAME = "pickUpLeft";

//Only points inside the box are relevant
tf2::Vector3 startBox = tf2::Vector3(0.0,0.0,0.005);
tf2::Vector3 endBox = tf2::Vector3(0.18,0.4,0.15);

//Used for hysteresis thresholding to find faces which are graspable
double thresholdA = 0.15;
double thresholdB = 0.7;

//All graspable faces closer than this will be merged
double radius = 0.0125;

//Used in color segmentation to merge points/regions with the same color
int regionColorThreshold = 5;
int pointColorThreshold = 6;
bool enableColorSegmentation = true;
//How many neighbours should be used to calculate the normal vector for each point?
int k = 20;
//What is the min and max size of points inside a graspable cluster?
int minClusterSize = 20;
int maxClusterSize = 3000;
//All non graspable points which are closer than this to a graspable surface will be merged with that surface
double regionDistanceThreshold = 0.02;

std::string cameraTopic = "kinect2_jetson/hd/points";

//Used to detect the background, which can be filtered or merged with other clusters.
double backgroundFilterThreshold = 30.0;
int backgroundR = 0;
int backgroundG = 0;
int backgroundB = 0;
bool cutAwayBackbround = false;

State::State currentState = State::INIT;
tf2_ros::Buffer tfBuffer;
std::unique_ptr<MarkerHelper> markerManager;
std::shared_ptr<ros::NodeHandle> nh;
pcl::PointCloud<cloud::PointType>::Ptr currentScene;
ros::Time sceneTime;

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
//Detects the yellow and red markers and calculates the truck rotation.
bool getTruckRotation(pcl::PointCloud<cloud::PointType>::Ptr scene, const double yellowThreshold, const double redThreshold, tf2::Quaternion& rot,
		pcl::PointCloud<cloud::PointType>::Ptr red, pcl::PointCloud<cloud::PointType>::Ptr yellow, std::vector<tf2::Vector3>& borderPoints)
{
	cloud::PointType yellowPoint;
	yellowPoint.r = 238; yellowPoint.g = 228; yellowPoint.b = 45;
	cloud::findRegionWithColor(scene,yellow,yellowThreshold,yellowPoint);
	std::cout << "yellow points: " << yellow->size() << std::endl;

	cloud::PointType redPoint;
	redPoint.r = 163; redPoint.g = 39; redPoint.b = 61;
	cloud::findRegionWithColor(scene,red,redThreshold,redPoint);
	std::cout << "red points: " << red->size() << std::endl;

	if(yellow->size() > 0 && red->size() > 0)
	{
		tf2::Vector3 aR2d = tf2::Vector3(0,0,0);
		tf2::Vector3 aY2d = tf2::Vector3(0,0,0);
		tf2::Vector3 aY3d = tf2::Vector3(0,0,0);
		tf2::Vector3 aR3d = tf2::Vector3(0,0,0);
		double maxDistance = -1.0;
		for(size_t i = 0; i < yellow->size(); i++)
		{
			for(size_t j = 0; j < red->size(); j++)
			{
				tf2::Vector3 y(yellow->at(i).x,yellow->at(i).y,0);
				tf2::Vector3 r(red->at(j).x,red->at(j).y,0);
				double d = y.distance2(r);
				if(d > maxDistance || maxDistance < 0)
				{
					aR2d = r; aY2d = y;
					aY3d = tf2::Vector3(yellow->at(i).x,yellow->at(i).y,yellow->at(i).z);
					aR3d = tf2::Vector3(red->at(j).x,red->at(j).y,red->at(j).z);
					maxDistance = d;
				}
			}
		}

		tf2::Vector3 bY2d = tf2::Vector3(0,0,0);
		tf2::Vector3 bY3d = tf2::Vector3(0,0,0);
		maxDistance = -1.0;
		for(size_t i = 0; i < yellow->size(); i++)
		{
			tf2::Vector3 y(yellow->at(i).x,yellow->at(i).y,0);
			double d = y.distance(aR2d) + y.distance(aY2d);
			if(d > maxDistance || maxDistance < 0)
			{
				bY2d = y;
				bY3d = tf2::Vector3(yellow->at(i).x,yellow->at(i).y,yellow->at(i).z);
				maxDistance = d;
			}
		}

		tf2::Vector3 bR2d = tf2::Vector3(0,0,0);
		tf2::Vector3 bR3d = tf2::Vector3(0,0,0);
		maxDistance = -1.0;
		for(size_t i = 0; i < red->size(); i++)
		{
			tf2::Vector3 r(red->at(i).x,red->at(i).y,0);
			double d = r.distance(aR2d) + r.distance(aY2d);
			if(d > maxDistance || maxDistance < 0)
			{
				bR2d = r;
				bR3d = tf2::Vector3(red->at(i).x,red->at(i).y,red->at(i).z);
				maxDistance = d;
			}
		}

		//We have 4 points. Lets build two vectors which go from one to another point and are parallel to each other. Thats the truck rotation.
		tf2::Vector3 normalA = aY2d - bR2d;
		normalA.normalize();
		tf2::Vector3 normalB = bY2d - aR2d;
		normalB.normalize();

		tf2::Vector3 forwardVector = (normalA + normalB) / 2.0;
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
		borderPoints.push_back(bR3d);
		borderPoints.push_back(bY3d);
		borderPoints.push_back(aR3d);
		return true;
	}
	return false;
}
//Finds four points which have the highest distance to each other. Based on them, the rotation is detected.
bool getBlockRotation(pcl::PointCloud<cloud::PointType>::Ptr scene, tf2::Quaternion& rot, std::vector<tf2::Vector3>& borderPoints)
{
	if(scene->size() <= 3 ) return false;

	//2d is used for rotation calculation, 3d is used to calculate the objects dimenions
	tf2::Vector3 aY2d = tf2::Vector3(0,0,0);
	tf2::Vector3 bY2d = tf2::Vector3(0,0,0);
	tf2::Vector3 aY3d = tf2::Vector3(0,0,0);
	tf2::Vector3 bY3d = tf2::Vector3(0,0,0);
	double maxDistance = -1.0;
	for(size_t i = 0; i < scene->size(); i++)//Finds two border points (the border points are the two points which are the farthest away of each other.)
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

	//2d is used for rotation calculation, 3d is used to calculate the objects dimenions
	tf2::Vector3 aR2d = tf2::Vector3(0,0,0);
	tf2::Vector3 bR2d = tf2::Vector3(0,0,0);
	tf2::Vector3 aR3d = tf2::Vector3(0,0,0);
	tf2::Vector3 bR3d = tf2::Vector3(0,0,0);
	maxDistance = -1.0;
	for(size_t i = 0; i < scene->size(); i++)//Finds the other two border points, which are the farthest away of each other and of the first two border points
	{
		for(size_t j = 0; j < scene->size(); j++)
		{
			if(i == j) continue;
			tf2::Vector3 a(scene->at(i).x,scene->at(i).y,0);
			tf2::Vector3 b(scene->at(j).x,scene->at(j).y,0);
			double d = a.distance(b) + a.distance(aY2d) + a.distance(bY2d) + b.distance(aY2d) + b.distance(bY2d);
			if(d > maxDistance || maxDistance < 0)
			{
				aR2d = a; bR2d = b;
				aR3d = tf2::Vector3(scene->at(i).x,scene->at(i).y,scene->at(i).z);
				bR3d = tf2::Vector3(scene->at(j).x,scene->at(j).y,scene->at(j).z);
				maxDistance = d;
			}
		}
	}

	//Find two vectors, which are parallel to each other. These point in the block direction.
	tf2::Vector3 normalEvenA = aY2d - aR2d;
	normalEvenA.normalize();
	tf2::Vector3 normalEvenB = bY2d - bR2d;
	normalEvenB.normalize();

	//Lets average the forward vector, because we actually have two independent normal vectors which can be used for rotation estimation
	tf2::Vector3 forwardVector;
	forwardVector = (normalEvenA + normalEvenB) / 2.0;
	forwardVector.normalize();

	//This calculates the angle between two vectors. The global forward vector and the blocks forward vector
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

//An object consists of more than only the grasp surface. Lets grow the grasp surface larger and add neighbouring points within range.
void getClusterSurrounding(pcl::PointCloud<cloud::PointType>::Ptr cutSzene, std::vector<pcl::PointCloud<cloud::PointType>::Ptr> graspPointClusters,
		std::vector<pcl::PointCloud<cloud::PointType>::Ptr> & cloudsExtended)
{
	//pcl::VoxelGrid<cloud::PointType> sor;
	//sor.setInputCloud(cutSzene);
	//sor.setLeafSize(0.0025f,0.0025f,0.0025f);
	pcl::PointCloud<cloud::PointType>::Ptr cutSceneVox = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
	//sor.filter(*cutSceneVox);
	*cutSceneVox = *cutSzene;

	//Currently clouds only stores possible grasp surfaces. But the object also might contain non graspable places, which still belong to the object.
	//Clouds extended will hold all neighbouring points within range close to a graspable surface.
	//Distances stores the distance for each point to other clusters
	std::vector<std::vector<double>> distances(graspPointClusters.size());
	for(size_t i = 0; i < graspPointClusters.size(); i++)
	{
		pcl::search::KdTree<cloud::PointType> tree;
		tree.setInputCloud(graspPointClusters.at(i));
		distances.at(i).resize(cutSceneVox->size());
		for(size_t j = 0; j < cutSceneVox->size(); j++)
		{
			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);
			distances.at(i).at(j) = -1.0;
			if(tree.nearestKSearch(cutSceneVox->at(j),1,pointIdxNKNSearch,pointNKNSquaredDistance))
			{
				if(std::sqrt(pointNKNSquaredDistance.at(0)) < regionDistanceThreshold)
				{
					distances.at(i).at(j) = pointNKNSquaredDistance.at(0);
				}
			}
		}
	}
	cloudsExtended.resize(graspPointClusters.size());
	for(size_t k = 0; k < cloudsExtended.size(); k++)
	{
		cloudsExtended.at(k) = boost::make_shared<pcl::PointCloud<cloud::PointType>>();;
	}

	for(size_t p = 0; p < cutSceneVox->size(); p++)
	{
		int bestSegment = -1;
		double minDistance = 0.0;
		for(size_t q = 0; q < graspPointClusters.size(); q++)
		{
			if((bestSegment == -1) || (minDistance > distances.at(q).at(p)))
			{
				if(distances.at(q).at(p) >= 0.0)
				{
					minDistance = distances.at(q).at(p);
					bestSegment = (int)q;
				}
			}
		}
		if(bestSegment >= 0.0)
		{
			cloudsExtended.at(bestSegment)->push_back(cutSceneVox->at(p));
		}
	}
}

int main (int argc, char** argv)
{
	std::string applicationName = "graspPointDetection";
	if(argc > 1) applicationName = argv[1];
	else std::cout << "Usage: rosrun detection graspPointDetection [NodeName] [TargetFrame]\n";
	if(argc > 2) TARGET_FRAME = argv[2];

	ros::init (argc, argv, "~");
	nh = std::make_shared<ros::NodeHandle>();

	ros::Publisher objectsPub = nh->advertise<detection_msgs::DetectedObjects>("/" + applicationName + "/objects",10);
	ros::Publisher debugGraspPub = nh->advertise<sensor_msgs::PointCloud2>("/" + applicationName + "/debugGrasp",2);
	ros::Publisher debugClusterPub = nh->advertise<sensor_msgs::PointCloud2>("/" + applicationName + "/debugCluster",2);
	ros::Publisher debugRotationPub = nh->advertise<sensor_msgs::PointCloud2>("/" + applicationName + "/debugRotation",2);

	double yellowThreshold = 25;
	double redThreshold = 20;
	double distanceThreshold = 0.01;
	bool detectTruckRotation = true;

	ros::Duration(0.5).sleep();

	nh->param("/" + applicationName + "/thresholdA",thresholdA);
	nh->param("/" + applicationName + "/thresholdB",thresholdB);
	nh->param("/" + applicationName + "/radius",regionDistanceThreshold);
	nh->param("/" + applicationName + "/colorSegmentation",enableColorSegmentation);
	nh->param("/" + applicationName + "/startX",startBox.x());
	nh->param("/" + applicationName + "/startY",startBox.y());
	nh->param("/" + applicationName + "/startZ",startBox.z());
	nh->param("/" + applicationName + "/endX",endBox.x());
	nh->param("/" + applicationName + "/endY",endBox.y());
	nh->param("/" + applicationName + "/endZ",endBox.z());
	nh->param("/" + applicationName + "/pointColorThreshold",pointColorThreshold);
	nh->param("/" + applicationName + "/regionColorThreshold",regionColorThreshold);
	nh->param("/" + applicationName + "/minClusterSize",minClusterSize);
	nh->param("/" + applicationName + "/maxClusterSize",maxClusterSize);
	nh->param("/" + applicationName + "/distanceThreshold",distanceThreshold);
	nh->param("/" + applicationName + "/detectTruckRotation",detectTruckRotation);

	nh->param("/" + applicationName + "/backgroundColor/r",backgroundR);
	nh->param("/" + applicationName + "/backgroundColor/g",backgroundG);
	nh->param("/" + applicationName + "/backgroundColor/b",backgroundB);
	nh->param("/" + applicationName + "/backgroundFilterThreshold",backgroundFilterThreshold);
	nh->param("/" + applicationName + "/cutAwayBackbround",cutAwayBackbround);
	nh->param("/" + applicationName + "/yellowThreshold",yellowThreshold);
	nh->param("/" + applicationName + "/redThreshold",redThreshold);
	nh->param("/" + applicationName + "/cameraTopic",cameraTopic);


	ros::Subscriber sub;

	currentScene = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
	tf2_ros::TransformListener tfListener(tfBuffer);
	markerManager = std::make_unique<MarkerHelper>(nh,"/" + applicationName + "/marker","objects");

	sub = nh->subscribe<sensor_msgs::PointCloud2> (cameraTopic, 1, pointcloudCallback);

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
				nh->getParam("/" + applicationName + "/thresholdA",thresholdA);
				nh->getParam("/" + applicationName + "/thresholdB",thresholdB);
				nh->getParam("/" + applicationName + "/radius",regionDistanceThreshold);
				nh->getParam("/" + applicationName + "/colorSegmentation",enableColorSegmentation);
				double x1,y1,z1,x2,y2,z2;
				nh->getParam("/" + applicationName + "/startX",x1);
				nh->getParam("/" + applicationName + "/startY",y1);
				nh->getParam("/" + applicationName + "/startZ",z1);
				nh->getParam("/" + applicationName + "/endX",x2);
				nh->getParam("/" + applicationName + "/endY",y2);
				nh->getParam("/" + applicationName + "/endZ",z2);
				startBox = tf2::Vector3(x1,y1,z1);
				endBox = tf2::Vector3(x2,y2,z2);

				nh->getParam("/" + applicationName + "/pointColorThreshold",pointColorThreshold);
				nh->getParam("/" + applicationName + "/regionColorThreshold",regionColorThreshold);
				nh->getParam("/" + applicationName + "/minClusterSize",minClusterSize);
				nh->getParam("/" + applicationName + "/maxClusterSize",maxClusterSize);
				nh->getParam("/" + applicationName + "/distanceThreshold",distanceThreshold);
				nh->getParam("/" + applicationName + "/detectTruckRotation",detectTruckRotation);

				nh->getParam("/" + applicationName + "/backgroundColor/r",backgroundR);
				nh->getParam("/" + applicationName + "/backgroundColor/g",backgroundG);
				nh->getParam("/" + applicationName + "/backgroundColor/b",backgroundB);
				nh->getParam("/" + applicationName + "/backgroundFilterThreshold",backgroundFilterThreshold);
				nh->getParam("/" + applicationName + "/cutAwayBackbround",cutAwayBackbround);
				nh->getParam("/" + applicationName + "/yellowThreshold",yellowThreshold);
				nh->getParam("/" + applicationName + "/redThreshold",redThreshold);

				//Using a box cut to only keep points inside the pickup area.
				pcl::PointCloud<cloud::PointType>::Ptr cutScene = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
				cloud::cutPointCloudRect(currentScene,cutScene,startBox,endBox);

				pcl::PointCloud<cloud::PointType>::Ptr cutSceneRGB = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
				if(backgroundFilterThreshold > 0 && cutAwayBackbround)
				{
					cloud::cutBackgroundColor(cutScene,cutSceneRGB,backgroundFilterThreshold,tf2::Vector3((double)backgroundR,(double)backgroundG,(double)backgroundB));
				}else{
					*cutSceneRGB = *cutScene;
				}

				std::cout << "Input points: " << cutSceneRGB->size() << std::endl;
				//Using normal segmentation to only keep points which face upwoards and therefore can be grasped.
				pcl::PointCloud<cloud::PointType>::Ptr normalSeg = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
				cloud::normalBasedSegmentation(cutSceneRGB,normalSeg,tf2::Vector3(0,0,1),thresholdA,thresholdB,radius,k);

				pcl::PointCloud<cloud::PointType>::Ptr colorCloud = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
				std::vector<pcl::PointCloud<cloud::PointType>::Ptr> clouds;

				if(enableColorSegmentation)
				{
					//Using color region growing to divide areas of different color into different pointclouds
					cloud::colorBasedRegionGrowing(normalSeg,colorCloud,clouds,minClusterSize,maxClusterSize,distanceThreshold,pointColorThreshold,regionColorThreshold);
				}else{
					cloud::colorBasedRegionGrowing(normalSeg,colorCloud,clouds,minClusterSize,maxClusterSize,distanceThreshold,INT_MAX,INT_MAX);
				}

				if(debugGraspPub.getNumSubscribers() > 0)
				{
					sensor_msgs::PointCloud2 debugCloudOut;
					pcl::toROSMsg(*colorCloud, debugCloudOut); //colorCloud
					debugCloudOut.header.stamp = ros::Time::now();
					debugCloudOut.header.frame_id = TARGET_FRAME;
					debugGraspPub.publish(debugCloudOut);
				}

				//Right now we only have grasp points. But objects also have side surfaes and uneven surfaces. Lets extend the clusters by this.
				std::vector<pcl::PointCloud<cloud::PointType>::Ptr> cloudsExtended;
				getClusterSurrounding(cutSceneRGB,clouds,cloudsExtended);

				//Sending cloud for visualization
				if(debugClusterPub.getNumSubscribers() > 0)
				{
					colorCloud->clear();
					for(size_t n = 0; n < cloudsExtended.size(); n++)
					{
						pcl::PointCloud<cloud::PointType>::Ptr c = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
						for(cloud::PointType p : *cloudsExtended.at(n))
						{
							int r = n % 4;
							int g = (n/2) % 4;
							int b = (n/4) % 4;

							p.r = (r*255)/3;
							p.g = (g*255)/3;
							p.b = (b*255)/3;
							colorCloud->push_back(p);
						}
					}

					sensor_msgs::PointCloud2 debugCloudOut;
					pcl::toROSMsg(*colorCloud, debugCloudOut); //colorCloud
					debugCloudOut.header.stamp = ros::Time::now();
					debugCloudOut.header.frame_id = TARGET_FRAME;
					debugClusterPub.publish(debugCloudOut);
				}

				std::cout << "Creating messages!\n";
				int counter = 0;
				//viewer.removeAllShapes(0);
				std::vector<detection_msgs::DetectedObject> objects(clouds.size());
				//Find the median point for all graspable surfaces
				#pragma omp parallel for shared(objects)
				for(size_t i = 0; i < clouds.size(); i++)
				{
					pcl::PointCloud<cloud::PointType>::Ptr c = clouds.at(i);
					size_t medianPointId = 0;
					cloud::getMedianPoint(c,medianPointId);

					tf2::Quaternion partRotation = tf2::Quaternion(0,0,0,1);
					std::vector<tf2::Vector3> borderPoints;
					if(detectTruckRotation) //Lets detect the objects orientation. Yellow and red stripes need to be on the object!
					{
						pcl::PointCloud<cloud::PointType>::Ptr red = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
						pcl::PointCloud<cloud::PointType>::Ptr yellow = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
						if(!getTruckRotation(cloudsExtended.at(i),yellowThreshold,redThreshold,partRotation,red,yellow,borderPoints))
						{
							partRotation = tf2::Quaternion(0,0,0,1);
						}else{
							if(debugRotationPub.getNumSubscribers() > 0)
							{
								sensor_msgs::PointCloud2 debugCloud2Out;
								pcl::toROSMsg(*red + *yellow, debugCloud2Out);
								debugCloud2Out.header.stamp = ros::Time::now();
								debugCloud2Out.header.frame_id = TARGET_FRAME;
								debugRotationPub.publish(debugCloud2Out);
							}
						}
					}else{
						if(!getBlockRotation(cloudsExtended.at(i),partRotation,borderPoints))
						{
							partRotation = tf2::Quaternion(0,0,0,1);
						}
					}

					if(partRotation.x() != partRotation.x() || partRotation.y() != partRotation.y() ||
							partRotation.z() != partRotation.z() || partRotation.w() != partRotation.w())
					{
						continue;
					}

					pcl::PointXYZ start(0,0,0);
					pcl::PointXYZ end(0,0,0);
					double height = 0.0;
					cloud::getEndAndStartPoint2D(clouds.at(i),start,end,height);

					detection_msgs::DetectedObject obj;
					obj.rotation = tf2::toMsg(partRotation);
					obj.graspPoint.x = c->at(medianPointId).x;
					obj.graspPoint.y = c->at(medianPointId).y;
					obj.graspPoint.z = c->at(medianPointId).z;

					if(borderPoints.size() == 4)
					{
						markerManager->publishArrow(tf2::Transform(partRotation,borderPoints.at(0)),
								TARGET_FRAME,42,0.05,0,255,0,"rotation",5.0);
						markerManager->publishArrow(tf2::Transform(partRotation,borderPoints.at(1)),
														TARGET_FRAME,10 + i,0.05,0,255,0,"rotation",5.0);
						markerManager->publishArrow(tf2::Transform(partRotation,borderPoints.at(2)),
														TARGET_FRAME,20 + i,0.05,0,255,0,"rotation",5.0);
						markerManager->publishArrow(tf2::Transform(partRotation,borderPoints.at(3)),
														TARGET_FRAME,30 + i,0.05,0,255,0,"rotation",5.0);

						obj.startPoint.x = borderPoints.at(0).x();
						obj.startPoint.y = borderPoints.at(0).y();
						obj.startPoint.z = 0.0; //We know that each object lies on the ground. So min z=0.0
						obj.endPoint.x = borderPoints.at(2).x();
						obj.endPoint.y = borderPoints.at(2).y();

						if(detectTruckRotation)
						{
							tf2::Vector3 a = (borderPoints.at(0) + borderPoints.at(1))/2.0;
							tf2::Vector3 b = (borderPoints.at(2) + borderPoints.at(3))/2.0;
							tf2::Vector3 center = (a + b)/2.0;

							obj.graspPoint.x = center.x();
							obj.graspPoint.y = center.y();
							obj.graspPoint.z = center.z();
						}

					}else{
						obj.startPoint.x = start.x;
						obj.startPoint.y = start.y;
						obj.startPoint.z = 0.0; //We know that each object lies on the ground. So min z=0.0
						obj.endPoint.x = end.x;
						obj.endPoint.y = end.y;
					}
					obj.endPoint.z = height;

					obj.color.resize(3);
					obj.color.at(0) = float(c->at(medianPointId).r)/255.0F;
					obj.color.at(1) = float(c->at(medianPointId).g)/255.0F;
					obj.color.at(2) = float(c->at(medianPointId).b)/255.0F;
					obj.timestamp = sceneTime;

					objects.at(i) = obj;

					markerManager->publishMarker(obj.endPoint,TARGET_FRAME,i+50,255,0,0,"test",1.0);
					markerManager->publishMarker(obj.startPoint,TARGET_FRAME,i+100,0,255,0,"test",1.0);
				}

				std::cout << "Sending messages!\n";
				detection_msgs::DetectedObjects msg;
				msg.objects = objects;
				msg.frame_id = TARGET_FRAME;
				objectsPub.publish(msg);

				//Drawing graps points on viewer. Publishing visualization_msgs for visualization in rviz
				for(auto obj : objects)
				{
					markerManager->publishMarker(obj.graspPoint,TARGET_FRAME,counter,1,1,0,"graspPoint",1.0);
					counter++;
				}

				markerManager->publishBox(startBox,endBox,TARGET_FRAME,0,0,1.0,0,0.33,"BoundingBox",0);

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
