/**
 * tfPublisherClient.cpp
 *
 *  Created on: Mai 27, 2019
 *      Author: philipf
 *
 *      This plugin publishes a world frame via tf. You can set the transform from the 3d sensor to the world frame by using
 *      a controller. Moving the knobs and the arrow keys on the controller will move the camera frame. By using LB and RB you
 *      can set the speed. By using LT and RB you can set the size of the frame (for better visuality). The A button will apply the current
 *      location of the frame, to be used as the new world frame. X will reset the coordinate frame rotation. Y will update the pointcloud.
 *      B will remove all keypoints and coordinate frames. Start button will save the current settings into /settings/cameraFrame.settings and
 *      can be loaded automatically on the next restart. Back button will reset the current frame and move into the last published frame.
 *
 *      Press the left knob to move into the nearest point. Press right knob to move into nearest point and allign with its rotation.
 */
#include <iostream>
#include <string>
#include <fstream>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>

#include <sensor_msgs/Joy.h>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Vector3.h>

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
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/board.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <detection_msgs/AddFrame.h>
#include <detection_msgs/SetFrame.h>
#include <detection_msgs/GetFrames.h>
#include <detection_msgs/SetString.h>
#include <detection_msgs/SetPointcloud.h>
#include <std_srvs/Empty.h>
#include "../include/cloud.h"

struct Frame
{
	Frame(std::string gName="") : name(gName)
	{
		transform = tf2::Transform(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));
		dirty = false;
	}
	std::string name;
	tf2::Transform transform;
	//Is the frame modified and needs to be send to the server?
	bool dirty;
};

const float UPDATE_RATE = 24.0;
/**
 * Definition of speeds for moving the frame.
 */
const double JOY_SPEED_NORMAL = 0.005;
const double JOY_SPEED_SLOW = JOY_SPEED_NORMAL * 0.025;
const double JOY_SPEED_FAST = JOY_SPEED_NORMAL * 5;
//Number of samples which will be combined into one shot. More samples reduce noise, but cost more computation time.
const size_t MAX_NUM_SAMPLES = 8;
/**
 *  Helper variables used later.
 */
tf2::Transform lastSendTransform;
tf2::Vector3 rotationEstimationSettings;
boost::shared_ptr<ros::NodeHandle> nh;
pcl::visualization::PCLVisualizer::Ptr viewer;
tf2_ros::Buffer tfBuffer;
pcl::PointCloud<cloud::PointType>::Ptr sampledCloud;
pcl::PointCloud<cloud::PointType>::Ptr sampledCloud2;
bool showSecondCamera = true;
sensor_msgs::JoyConstPtr joyMessage;
bool joyMessageDitry = false;
ros::ServiceClient addFrame;
ros::ServiceClient setFrame;
ros::ServiceClient getFrames;
ros::ServiceClient deleteFrame;
ros::ServiceClient saveFrames;
ros::ServiceClient registerPointcloudSubscriberJetson;
ros::ServiceClient registerPointcloudSubscriberXavier;
//Represents the current processed state of the joystick
bool buttonsProcessed[12] = {false,false,false,false,false,false,false,false,false,false,false,false};

//Default scale of the coordinate frame
double coordinateScale = 0.05;

/*All know frames and the current selected frame.*/
std::vector<Frame> frames;
unsigned int currentFrame = 0;
size_t serverFramesCount;
/**
 * Joystick callback.
 * @param joy
 */
void joyCallback (const sensor_msgs::JoyConstPtr &joy)
{
	joyMessage = joy;
	joyMessageDitry = true;
}
/**
 * Checks which transforms are currently available by the server
 */
bool getCurrentFrames()
{
	detection_msgs::GetFrames::Request req;
	detection_msgs::GetFrames::Response res;
	if(!getFrames.call(req,res)){
		ROS_ERROR("tfPublisher could not be reached!");
		return false;
	}

	std::vector<std::string> tmp;
	tfBuffer._getFrameStrings(tmp);
	serverFramesCount = tmp.size();

	std::cout << "Server returned " << res.names.size() << " frames!\n";

	std::vector<bool> valid(frames.size());
	for(size_t n = 0; n < valid.size(); n++) valid.at(n) = false;

	for(size_t j = 0; j < res.names.size(); j++)
	{
		tf2::Transform t;
		t.setRotation(tf2::Quaternion(res.transforms.at(j).rotation.x,res.transforms.at(j).rotation.y,res.transforms.at(j).rotation.z,res.transforms.at(j).rotation.w));
		t.setOrigin(tf2::Vector3(res.transforms.at(j).translation.x,res.transforms.at(j).translation.y,res.transforms.at(j).translation.z));
		bool found = false;
		for(size_t i = 0; (i < frames.size()) && !found; i++)
		{
			if(frames.at(i).name == res.names.at(j))
			{
				if(!frames.at(i).dirty)
				{
					frames.at(i).transform = t;
					std::cout << "Updating frame: " << res.names.at(j) << std::endl;
				}
				valid.at(i) = true;
				found = true;
			}
		}
		if(!found)
		{
			Frame f;
			f.name = res.names.at(j);
			f.transform = t;
			f.dirty = false;
			frames.push_back(f);
			valid.push_back(true);
			std::cout << "Adding new frame: " << res.names.at(j) << std::endl;
		}
	}
	if(valid.size() > 0)
	{
		for(size_t i = 0; i < valid.size(); i++)
		{
			int index = valid.size() - i - 1;
			if(!valid.at(index))
			{
				viewer->removeCoordinateSystem(frames.at(index).name,0);
				viewer->removeShape(frames.at(index).name + "_text",0);
				std::cout << "Removing frame: " << frames.at(index).name << std::endl;
				frames.erase(frames.begin() + index);
			}
		}
	}
	return true;
}

/**
 * Will calculate the reference frame of point "id" in the given pointcloud and sets foundRot.
 * @param id Id of the selected point from cloudRGB
 * @param cloudRGB Pointcloud where reference frame should be estimated.
 * @param foundRot The found rotation of the reference frame will be stored here.
 * @return If some pcl methods fail, this will be false.
 */
bool adaptPointRotation(size_t id, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB, tf2::Quaternion &foundRot)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr rfs = boost::make_shared<pcl::PointCloud<pcl::ReferenceFrame>>();
	pcl::copyPointCloud(*cloudRGB,*cloud);

	std::cout << "[tfPublisherClient]: Estimating normals!" << std::endl;
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setInputCloud (cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch(5);
	norm_est.compute (*normals);

	std::cout << "[tfPublisherClient]: Done estimating normals!" << std::endl;

	//Adding the selected point to the list of keypoints, where reference frames have to be estimated.
	pcl::PointXYZ point;
	point.x = cloud->at(id).x;
	point.y = cloud->at(id).y;
	point.z = cloud->at(id).z;
	keypoints->push_back(point);

	std::cout << "[tfPublisherClient]: Estimating reference frames!" << std::endl;
	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
	rf_est.setFindHoles (true);
	rf_est.setRadiusSearch (75 * 0.000680933);
	rf_est.setInputCloud (keypoints);
	rf_est.setInputNormals (normals);
	rf_est.setSearchSurface (cloud);
	rf_est.compute (*rfs);

	//Extract the reference frame and store it into the quaternion.
	std::cout << "[tfPublisherClient]: Found rfs: " << rfs->size() << " id was: "<< 0 << std::endl;
	if(rfs->size() >= 1)
	{
		pcl::ReferenceFrame frame = rfs->at(0);
		tf2::Vector3 zVec(frame.rf[6],frame.rf[7],frame.rf[8]);
		tf2::Vector3 xVec(1,0,0);
		tf2::Vector3 yVec = zVec.cross(xVec);

		frame.rf[0] = xVec.x(); frame.rf[1] = xVec.y(); frame.rf[2] = xVec.z();
		frame.rf[3] = yVec.x(); frame.rf[4] = yVec.y(); frame.rf[5] = yVec.z();

		std::cout << "[tfPublisherClient]: Getting coordinate Frame!" << std::endl;
		Eigen::Map<Eigen::Matrix3f> rot = frame.getMatrix3fMap(); //rfs->at(newId).getMatrix3fMap();
		Eigen::Quaternion<float> q(rot);
		foundRot = tf2::Quaternion(q.x(),q.y(),q.z(),q.w());
		return true;
	}
	else
	{
		std::cout << "[tfPublisherClient]: Failed to get reference frames!" << std::endl;
	}
	return false;
}
/**
 * Gets the closest point next to loc and sets Id and foundLoc.
 * @param loc Location where to find the nearest point.
 * @param cloudRGB Pointcloud where the nearest point has to be searched in
 * @param id This id will be set to the nearest points id.
 * @param foundLoc This vector will be set to the nearest points location.
 * @return returns false if pcl methods fail.
 */
bool getClosestPoint (tf2::Vector3 loc, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB, size_t &id, tf2::Vector3 &foundLoc)
{
	//using a kdtree to search for the nearest point
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::copyPointCloud(*cloudRGB,*tmpCloud);
	kdtree.setInputCloud (tmpCloud);
	pcl::PointXYZ searchPoint;
	searchPoint.x = loc.x();
	searchPoint.y = loc.y();
	searchPoint.z = loc.z();

	int K = 10;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	//Setting the vector to the nearest points location.
	std::cout << "[tfPublisherClient]: Searching nearest point!" << std::endl;
	if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		id = pointIdxNKNSearch[0];
		foundLoc.setX(tmpCloud->points[ pointIdxNKNSearch[0]].x);
		foundLoc.setY(tmpCloud->points[ pointIdxNKNSearch[0]].y);
		foundLoc.setZ(tmpCloud->points[ pointIdxNKNSearch[0]].z);
		std::cout << "[tfPublisherClient]: Found P[" << id << "]= (" << foundLoc.x() << "," << foundLoc.y() << "," << foundLoc.z() << ")"<< std::endl;
		pcl::PointXYZRGB rgbPoint;
		pcl::copyPoint(cloudRGB->points[pointIdxNKNSearch[0]],rgbPoint);
		std::cout << "[tfPublisherClient]: R:" << (short)rgbPoint.r << " G: " << (short)rgbPoint.g << " B: " << (short)rgbPoint.b << std::endl;
		return true;
	}
	return false;
}

/**
 * Redraws all coordinate frames and text!
 */
void drawOrigins()
{
	for(size_t i = 0; i < frames.size(); i++)
	{
		Frame f = frames.at(i);
		if(i != 0) //World coordinates is relative to camera. Other transforms are relative to /world! Need conversion!
		{
			f.transform = frames.at(0).transform * f.transform;
		}
		Eigen::Affine3d t;
		geometry_msgs::Transform gT;
		tf2::convert(f.transform,gT);
		t = tf2::transformToEigen(gT);
		Eigen::Affine3f affine = t.cast<float>();
		viewer->removeCoordinateSystem(f.name,0);
		viewer->addCoordinateSystem (coordinateScale, affine,f.name, 0);
		pcl::PointXYZ p;
		p.x = f.transform.getOrigin().x();
		p.y = f.transform.getOrigin().y();
		p.z = f.transform.getOrigin().z() - coordinateScale;
		viewer->removeShape(f.name+"_text",0);

		if(frames.at(currentFrame).dirty)
		{
			if(i == currentFrame)
			{
				viewer->addText3D(f.name,p,coordinateScale*0.5,1,1,0,f.name+"_text",0);
			}else
			{
				viewer->addText3D(f.name,p,coordinateScale*0.5,1,0,0,f.name+"_text",0);
			}
		}else
		{
			if(i == currentFrame)
			{
				viewer->addText3D(f.name,p,coordinateScale*0.5,1,1,1,f.name+"_text",0);
			}else
			{
				viewer->addText3D(f.name,p,coordinateScale*0.5,0,1,0,f.name+"_text",0);
			}
		}
	}
	Frame f = frames.at(currentFrame);
	viewer->removeShape("currentText",0);
	viewer->removeShape("currentText2",0);
	viewer->removeShape("currentText3",0);
	viewer->addText("Current Frame: " + f.name,0,60,15,1.0,1.0,1.0,"currentText",0);
	viewer->addText("Location: [" + std::to_string(f.transform.getOrigin().x()) + ", " + std::to_string(f.transform.getOrigin().y())
			+ ", " + std::to_string(f.transform.getOrigin().z()) + "]",0,40,15,1.0,1.0,1.0,"currentText2",0);
	tf2::Vector3 rot = f.transform.getRotation().getAxis();
	rot *= (180.0/M_PI);
	viewer->addText("Rotation: [" + std::to_string(rot.x()) + ", " + std::to_string(rot.y())
				+ ", " + std::to_string(rot.z()) + "]",0,20,15,1.0,1.0,1.0,"currentText3",0);
}

/**
 * Check each frame if it is modifed and publishes its transform to the server
 */
void sendAllDirtyFrames()
{
	//Publish the current frames!
	for(size_t i = 0; i < frames.size(); i++)//Publish all tf-frames
	{
		Frame &f = frames.at(i);
		if(f.name.size() == 0) continue;
		if(f.dirty)
		{
			detection_msgs::SetFrame::Request req;
			detection_msgs::SetFrame::Response res;
			req.name = f.name;
			req.t.translation.x = f.transform.getOrigin().x();
			req.t.translation.y = f.transform.getOrigin().y();
			req.t.translation.z = f.transform.getOrigin().z();
			req.t.rotation.x = f.transform.getRotation().x();
			req.t.rotation.y = f.transform.getRotation().y();
			req.t.rotation.z = f.transform.getRotation().z();
			req.t.rotation.w = f.transform.getRotation().w();
			setFrame.call(req,res);
			f.dirty = false;
		}
	}
}
/*Registers to the tfPublisher. He will then subscribe to pointcloud, do smoothing and sends back te result to the given callback.*/
void askForCloud()
{
	std::cout << "[tfPublisherClient]: Asking for new clouds!" << std::endl;
	detection_msgs::SetString srv;
	srv.request.msg = "/tfPublisherClient/getPointcloudJetson";
	registerPointcloudSubscriberJetson.call(srv);
	srv.request.msg = "/tfPublisherClient/getPointcloudXavier";
	registerPointcloudSubscriberXavier.call(srv);
}
/*Callback called when smoothed pointcloud is ready*/
bool askPointcloudCB(detection_msgs::SetPointcloud::Request &req, detection_msgs::SetPointcloud::Response &res)
{
	pcl::PCLPointCloud2 cloud2;
	pcl_conversions::toPCL(req.cloud,cloud2);
	pcl::fromPCLPointCloud2(cloud2,*sampledCloud);
	std::cout << "[tfPublisherClient]: Showing new cloud!\n";
	viewer->removePointCloud("current",0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (sampledCloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (sampledCloud, rgb, "current");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current");
	return true;
}
/*Callback called when smoothed pointcloud is ready*/
bool askPointcloudCB2(detection_msgs::SetPointcloud::Request &req, detection_msgs::SetPointcloud::Response &res)
{
	sensor_msgs::PointCloud2 other;
	geometry_msgs::TransformStamped tS;
	try{
		tS = tfBuffer.lookupTransform("kinect2_jetson_rgb_optical_frame","kinect2_xavier_rgb_optical_frame",ros::Time(0));
		tf2::doTransform(req.cloud,other,tS);
		pcl::fromROSMsg(other,*sampledCloud2);
	}catch(tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
		return false;
	}
	std::cout << "[tfPublisherClient]: Showing new cloud!\n";
	viewer->removePointCloud("current2",0);
	if(showSecondCamera)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (sampledCloud2);
		viewer->addPointCloud<pcl::PointXYZRGB> (sampledCloud2, rgb, "current2");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current2");
	}
	return true;
}

/**
 * Will be called in a frequent time with respect to UPDATE_RATE. Reads the last send joystick commands and reacts to them.
 * @param t
 */
void timerCallback (const ros::TimerEvent& t)
{
	if (!joyMessageDitry) return;

	double currentSpeed = JOY_SPEED_NORMAL;
	if(joyMessage->buttons[4] == 1)//Change of movement speed
	{
		currentSpeed = JOY_SPEED_SLOW;
	}
	else if(joyMessage->buttons[5] == 1)//Change of movement speed
	{
		currentSpeed = JOY_SPEED_FAST;
	}

	if(joyMessage->axes[5] < 0)//Change of corrdinate frame scale (visual)
	{
		coordinateScale = coordinateScale * 1.1;
	}
	if(joyMessage->axes[2] < 0)//Change of corrdinate frame scale (visual)
	{
		coordinateScale = coordinateScale * 0.9;
	}
	if(joyMessage->buttons[0] == 1)//Will apply the current transform and set this as the world frame transform.
	{
		if(!buttonsProcessed[0])
		{
			if(joyMessage->buttons[4])
			{
				if(currentFrame != 0) //World frame can not be deleted!
				{
					std::cout << "Removing frame: " << frames.at(currentFrame).name << std::endl;
					viewer->removeCoordinateSystem(frames.at(currentFrame).name,0);
					viewer->removeShape(frames.at(currentFrame).name + "_text",0);
					detection_msgs::AddFrame srv;
					srv.request.name = frames.at(currentFrame).name;
					deleteFrame.call(srv.request,srv.response);
				}
			}else{
				lastSendTransform = frames.at(currentFrame).transform;
				sendAllDirtyFrames();
			}
			buttonsProcessed[0] = true;
		}
	}else{
		buttonsProcessed[0] = false;
	}
	if(joyMessage->buttons[1] == 1)//Flip z-Axis or next frame
	{
		if(!buttonsProcessed[1])
		{
			if(joyMessage->buttons[4]) //Select next frame
			{
				currentFrame = (currentFrame+1)%frames.size();
				lastSendTransform = frames.at(currentFrame).transform;
				std::cout << "[tfPublisherClient]: Selecting frame " << currentFrame << ": " << frames.at(currentFrame).name << std::endl;
			}else //Flip axis
			{
				tf2::Quaternion rot = frames.at(currentFrame).transform.getRotation();
				tf2::Quaternion other(M_PI,0,0);
				rot = other * rot;
				frames.at(currentFrame).transform.setRotation(rot);
				frames.at(currentFrame).dirty = true;
			}
			buttonsProcessed[1] = true;
		}
	}else{
		buttonsProcessed[1] = false;
	}
	if(joyMessage->buttons[2] == 1)//Resets the current rotation or select previous frame
	{
		if(!buttonsProcessed[2])
		{
			if(joyMessage->buttons[4]) //Select previous frame
			{
				if(currentFrame == 0)
				{
					currentFrame = frames.size()-1;
				}else{
					currentFrame = (currentFrame-1)%frames.size();
				}
				lastSendTransform = frames.at(currentFrame).transform;
				std::cout << "[tfPublisherClient]: Selecting frame " << currentFrame << ": " << frames.at(currentFrame).name << std::endl;
			}
			else //Reset Rotation
			{
				std::cout << "[tfPublisherClient]: Reset rotation to [0 0 0 1]!" << std::endl;
				frames.at(currentFrame).transform.setRotation(tf2::Quaternion(0,0,0,1));
				frames.at(currentFrame).dirty = true;
			}
			buttonsProcessed[2] = true;
		}
	}else{
		buttonsProcessed[2] = false;
	}
	if((joyMessage->buttons[3] == 1) && nh)//Records a new pointcloud after pointcloud callback function has been called.
	{
		if(!buttonsProcessed[3])
		{
			if(joyMessage->buttons[4]) //hide second camera
			{
				showSecondCamera = !showSecondCamera;
				viewer->removePointCloud("current2",0);
				if(showSecondCamera)
				{
					pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (sampledCloud2);
					viewer->addPointCloud<pcl::PointXYZRGB> (sampledCloud2, rgb, "current2");
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current2");
				}
			}else{ //Select next frame
				askForCloud();
				getCurrentFrames();
			}
			buttonsProcessed[3] = true;
		}
	}else{
		buttonsProcessed[3] = false;
	}
	if(joyMessage->buttons[6] == 1) //Resets the full transform to the last send transform.
	{
		if(!buttonsProcessed[6])
		{
			std::cout << "[tfPublisherClient]: Reset to last published transform." << std::endl;
			frames.at(currentFrame).transform = lastSendTransform;
			buttonsProcessed[6] = true;
		}
	}else{
		buttonsProcessed[6] = false;
	}
	if(joyMessage->buttons[7] == 1)//Stores the current frames.at(currentFrame).transform into cameraFrame.settings file.
	{
		if(!buttonsProcessed[7])
		{
			sendAllDirtyFrames();
			std_srvs::Empty srv;
			saveFrames.call(srv);
			ROS_INFO("Saving all frames!");
			buttonsProcessed[7] = true;
		}
	}else{
		buttonsProcessed[7] = false;
	}
	if(joyMessage->buttons[10] == 1 || joyMessage->buttons[9] == 1) //Used to snap into the neares point or/and adapt its rotation.
	{
		if(!buttonsProcessed[10])
		{
			tf2::Vector3 currentPosition = frames.at(currentFrame).transform.getOrigin();
			if(currentFrame != 0)//Transform relative coordinates into camera coordinates:
			{
				currentPosition = (frames.at(0).transform * frames.at(currentFrame).transform).getOrigin();
			}
			tf2::Vector3 loc(0,0,0);
			tf2::Quaternion rot(0,0,0,1);
			size_t id;
			bool success = false;
			if(getClosestPoint(currentPosition,sampledCloud,id,loc))
			{
				if(joyMessage->buttons[10] == 1)
				{
					if(adaptPointRotation(id,sampledCloud,rot))
					{
						success = true; //Translation and rotation worked!
					}
				}
				else
				{
					success = true; //Translation worked! No rotation requested.
				}
			}
			if(success)
			{
				tf2::Transform t(rot,loc);
				if(currentFrame == 0) //World frame can use camera coordinates. Pointcloud alredady is in camera coordinates.
				{
					frames.at(currentFrame).transform = t;
				}else //Other frames have to use relative coordinates towards world. Transfrom camera coordinates to relative world coordinates:
				{
					frames.at(currentFrame).transform = frames.at(0).transform.inverse() * t;
				}
				frames.at(currentFrame).dirty = true;
			}
			buttonsProcessed[10] = true;
		}
	}else{
		buttonsProcessed[10] = false;
	}

	//Used for moving and rotating the world frame.
	tf2::Vector3 deltaLocation;
	tf2::Quaternion rotation = frames.at(currentFrame).transform.getRotation ();
	tf2::Quaternion delta;
	tf2::Vector3 dR(0,0,0);

	dR.setX((currentSpeed * joyMessage->axes[6]));
	dR.setY((currentSpeed * joyMessage->axes[7]));
	dR.setZ((currentSpeed * (-joyMessage->axes[3])));

	delta.setEuler(dR.getX(),dR.getY(),dR.getZ());

	rotation = rotation * delta;
	rotation.normalize();

	deltaLocation.setX (currentSpeed * (joyMessage->axes[1]));
	deltaLocation.setY (-currentSpeed * (joyMessage->axes[0]));
	deltaLocation.setZ (currentSpeed * joyMessage->axes[4]);

	frames.at(currentFrame).transform.setOrigin (frames.at(currentFrame).transform.getOrigin () + deltaLocation);
	frames.at(currentFrame).transform.setRotation (rotation);
	if(dR.length2() > 0.0 || deltaLocation.length2() > 0.0)
	{
		frames.at(currentFrame).dirty = true;
	}

	//Draws the current transform fame into the pcl viewer.
	drawOrigins();
}

int main (int argc, char* argv[])
{
	//Init ROS, pointers and viewer
	ros::init (argc, argv, "tfPublisherClient");
	nh = boost::make_shared<ros::NodeHandle>();
	tf2_ros::TransformListener tfListener(tfBuffer);
	sampledCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	sampledCloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>();
	viewer->setBackgroundColor (0.1, 0.1, 0.1, 0);
	viewer->setWindowName ("tfPublisherClient");
	viewer->initCameraParameters ();

	//Subscribe to all topics and services
	setFrame = nh->serviceClient<detection_msgs::SetFrame>("/tfPublisher/setFrame");
	getFrames = nh->serviceClient<detection_msgs::GetFrames>("/tfPublisher/getFrames");
	deleteFrame = nh->serviceClient<detection_msgs::AddFrame>("/tfPublisher/deleteFrame");
	saveFrames = nh->serviceClient<std_srvs::Empty>("/tfPublisher/save");
	registerPointcloudSubscriberJetson =  nh->serviceClient<detection_msgs::SetString>("/kinect2_jetson/askForPointcloud");
	registerPointcloudSubscriberXavier =  nh->serviceClient<detection_msgs::SetString>("/kinect2_xavier/askForPointcloud");

	ros::ServiceServer getPointCloudServer = nh->advertiseService("/tfPublisherClient/getPointcloudJetson",askPointcloudCB);
	ros::ServiceServer getPointCloudServer2 = nh->advertiseService("/tfPublisherClient/getPointcloudXavier",askPointcloudCB2);

	//Recieve all frames and draw them
	ros::Time::sleepUntil(ros::Time(1));
	if(!getCurrentFrames()) return 0;
	std::cout << "Frames size: " << frames.size() << " current frame: " << currentFrame << std::endl;
	lastSendTransform = frames.at(currentFrame).transform;
	drawOrigins();
	askForCloud();

	ros::Subscriber sub2 = nh->subscribe<sensor_msgs::Joy> ("/joy", 2, joyCallback);

	//Creates a timer which will frequently check the last joystick command and reacts to it.
	ros::Timer timer = nh->createTimer (ros::Duration (1.0F / 30.0), timerCallback);

	ros::Rate r(UPDATE_RATE);
	std::cout << "[tfPublisherClient]: Starting loop." << std::endl;
	while (ros::ok())
	{
		//Check for new unknown frames
		std::vector<std::string> tmp;

		tfBuffer._getFrameStrings(tmp);
		if(serverFramesCount != tmp.size())
		{
			if(!getCurrentFrames()) return 0;
		}
		viewer->spinOnce();
		ros::spinOnce ();
		r.sleep();
	}
	std::cout << "[tfPublisherClient]: Quit." << std::endl;
	return 0;
}
