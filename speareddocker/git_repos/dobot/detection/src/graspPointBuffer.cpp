/*
 * graspPointBuffer.cpp
 *	Subscribes to grasp points and filters out not consistent or non stable points.
 *  Created on: May 2, 2019
 *      Author: philipf
 */

#include <memory>
#include <ros/ros.h>
#include <ros/time.h>
#include <detection_msgs/DetectedObject.h>
#include <detection_msgs/DetectedObjects.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dobot/dobot.h>
#include <marker_helper/markerHelper.h>

const double GROW_RATE = 1.35;
const double LOOSE_RATE = 0.5;
size_t QUEUE_SIZE_LENGTH = 7;

/*This is a structure which stores information about an object.*/
struct MapObject
{
	tf2::Vector3 position;
	tf2::Vector3 start;
	tf2::Vector3 end;
	tf2::Quaternion rotation;
	std::vector<tf2::Vector3> positions;
	std::vector<tf2::Vector3> startPoints;
	std::vector<tf2::Vector3> endPoints;
	std::vector<tf2::Quaternion> rotations;
	detection_msgs::DetectedObject obj;
	bool updated = true;

	MapObject()
	{

	}
	//If a detection_msgs::DetectecObject has been recieved, it can be directly used to create a MapObject.
	//The MapObject uses datatypes which can be easier used for processing than the detection_msgs::DetectedObject
	MapObject(const detection_msgs::DetectedObject o)
	{
		position = tf2::Vector3(o.graspPoint.x,o.graspPoint.y,o.graspPoint.z);
		tf2::fromMsg(o.rotation,rotation);
		positions.push_back(position);
		rotations.push_back(rotation);
		start = tf2::Vector3(o.startPoint.x,o.startPoint.y,o.startPoint.z);
		end = tf2::Vector3(o.endPoint.x,o.endPoint.y,o.endPoint.z);
		startPoints.push_back(start);
		endPoints.push_back(end);
		obj = o;
		obj.probability = 0.25;
	}
	//Gets the median Rotation.
	tf2::Quaternion getMedian(std::vector<tf2::Quaternion> rot, bool blockRotationModulo90Degree)
	{
		if(rot.size() == 0) return tf2::Quaternion(0,0,0,1);

		double minDistance = -1.0;
		double bestRotation = 0.0;
		size_t best = 0;
		for(size_t i = 0; i < rot.size(); i++)//For all rotations, find the one which is the closest to all others.
		{
			double distance = 0.0;
			//Check for NAN
			if(rot.at(i).x() != rot.at(i).x() || rot.at(i).y() != rot.at(i).y() || rot.at(i).z() != rot.at(i).z() || rot.at(i).w() != rot.at(i).w())
			{
				continue;
			}

			double rA,pA,yA;
			tf2::Matrix3x3(rot.at(i)).getRPY(rA,pA,yA);
			yA = std::fmod(yA, M_PI_2);

			for(size_t j = 0; j < rot.size(); j++)//Sum up the distance to all other rotations, to be able to evaluate, if it is the median.
			{
				if(i == j) continue;
				//Check for NAN
				if(rot.at(j).x() != rot.at(j).x() || rot.at(j).y() != rot.at(j).y() || rot.at(j).z() != rot.at(j).z() || rot.at(j).w() != rot.at(j).w())
				{
					continue;
				}

				if(blockRotationModulo90Degree)
				{
					double rB,pB,yB;
					tf2::Matrix3x3(rot.at(i)).getRPY(rB,pB,yB);
					yB = std::fmod(yB, M_PI_2);//It is impossible to detect the rotation of a symmetric cube. All 4 sides are the same, this means all 4 rotations are right.
					double d1 = std::fabs(yA - yB);
					double d2 = (M_PI_2 - d1);
					if(d1 < d2)//angle shortest path!
					{
						distance += d1;
					}else{
						distance += d2;
					}
				}else{
					distance += std::fabs(rot.at(i).angleShortestPath(rot.at(j)));
				}
			}
			if(minDistance < 0.0 || distance < minDistance)
			{
				best = i;
				minDistance = distance;
				bestRotation = yA;
			}
		}
		if(minDistance < 0.0)
		{
			return tf2::Quaternion(0,0,0,1);
		}
		if(blockRotationModulo90Degree)
		{
			return tf2::Quaternion(0,0,bestRotation);
		}else{
			return rot.at(best);
		}
	}
	tf2::Vector3 getMedian(std::vector<tf2::Vector3> points, size_t& pos)
	{
		if(points.size() == 0) return tf2::Vector3(0,0,0);
		double minDistance = -1.0;
		size_t best = 0;
		for(size_t i = 0; i < points.size(); i++)
		{
			double distance = 0.0;
			for(size_t j = 0; j < points.size(); j++)
			{
				if(i == j) continue;
				distance += std::fabs(points.at(i).x() - points.at(j).x()) + std::fabs(points.at(i).y() - points.at(j).y()) + std::fabs(points.at(i).z() - points.at(j).z());
			}
			if(minDistance < 0.0 || distance < minDistance)
			{
				best = i;
				minDistance = distance;
			}
		}
		pos = best;
		return points.at(best);
	}
	/*Checks if the detected object is this object, if yes, merge it with the current data.*/
	bool checkForMatchAndInsert(const detection_msgs::DetectedObject o, double maxDistance=0.01, bool blockRotationModulo90Degree=false)
	{
		tf2::Vector3 positionB = tf2::Vector3(o.graspPoint.x,o.graspPoint.y,o.graspPoint.z);
		if(position.distance(positionB) < maxDistance)
		{
			obj.probability *= GROW_RATE;
			obj.probability = std::min(obj.probability,1.0);

			positions.push_back(tf2::Vector3(o.graspPoint.x,o.graspPoint.y,o.graspPoint.z));
			startPoints.push_back(tf2::Vector3(o.startPoint.x,o.startPoint.y,o.startPoint.z));
			endPoints.push_back(tf2::Vector3(o.endPoint.x,o.endPoint.y,o.endPoint.z));
			rotations.push_back(tf2::Quaternion(o.rotation.x,o.rotation.y,o.rotation.z,o.rotation.w));
			while(positions.size() > QUEUE_SIZE_LENGTH)
			{
				positions.erase(positions.begin());
				startPoints.erase(startPoints.begin());
				endPoints.erase(endPoints.begin());
				rotations.erase(rotations.begin());
			}

			size_t best = 0;
			tf2::Vector3 newPosition = getMedian(positions,best);
			tf2::Quaternion newRotation = getMedian(rotations,blockRotationModulo90Degree);
			//If new position is not stable, degregate!
			checkForDegregation(newPosition,newRotation);
			position = newPosition;
			rotation = newRotation;
			end = endPoints.at(best);
			start = startPoints.at(best);
			obj.timestamp = o.timestamp;
			return true;
		}
		return false;
	}
	void checkForDegregation(tf2::Vector3 newPosition, tf2::Quaternion newRotation)
	{
		double d = position.distance2(newPosition);
		if(d > 0.005)
		{
			obj.probability -= d*25.0;
			obj.probability = std::max(obj.probability,0.0);
		}
		d = rotation.angleShortestPath(this->rotation);
		if(d > M_PI_2 * 0.1)
		{
			obj.probability -= d * 2.0;
			obj.probability = std::max(obj.probability,0.0);
		}
	}
	void degregate()
	{
		obj.probability *= LOOSE_RATE;
	}
	bool hasLowProbability()
	{
		return (obj.probability < 0.25);
	}
	detection_msgs::DetectedObject get()
	{
		obj.endPoint.x = end.x();
		obj.endPoint.y = end.y();
		obj.endPoint.z = end.z();

		obj.startPoint.x = start.x();
		obj.startPoint.y = start.y();
		obj.startPoint.z = start.z();

		obj.graspPoint.x = position.x();
		obj.graspPoint.y = position.y();
		obj.graspPoint.z = position.z();

		obj.rotation.x = rotation.x();
		obj.rotation.y = rotation.y();
		obj.rotation.z = rotation.z();
		obj.rotation.w = rotation.w();
		return obj;
	}
};

detection_msgs::DetectedObjects objectsToPublish;
std::vector<MapObject> objects;
bool init = true;
std::shared_ptr<ros::NodeHandle> nh;
std::unique_ptr<MarkerHelper> markerManager;
std::string TARGET_FRAME;
std::string childNodeName = "LoadGraspDetection";
double maxDistance = 0.02;
bool blockRotationModulo90Degree = false;

void publishBoxRepresentingObject(MapObject o, int k)
{
	tf2::Vector3 end = o.end * tf2::Vector3(1,1,0);
	tf2::Vector3 start = o.start * tf2::Vector3(1,1,0);

	double length = (end-start).length();
	double width = length;
	double height = std::fabs(o.end.z() - o.start.z());
	markerManager->publishBox(tf2::Transform(o.rotation,o.position - tf2::Vector3(0,0,height*0.5)),TARGET_FRAME,
			tf2::Vector3(length,width,height),k,o.obj.color.at(0),o.obj.color.at(1),o.obj.color.at(2),"_box",2.0);
}

void aggregateObjects(detection_msgs::DetectedObjects other)
{
	TARGET_FRAME = other.frame_id;

	if(init)//Initialization. There is nothing to aggregate jet.
	{
		for(size_t i = 0; i < other.objects.size(); i++)
		{
			objects.push_back(MapObject(other.objects.at(i)));
			markerManager->publishMarker(other.objects.at(i).graspPoint,TARGET_FRAME,i,255,255,0,"_bad");
		}
		objectsToPublish.frame_id = other.frame_id;
		init = false;
		ROS_INFO("Init objects array!");
		return;
	}

	for(MapObject& o : objects)
	{
		o.updated = false;
	}

	for(size_t i = 0; i < other.objects.size(); i++) //Check which grasp points are already found. Check which ones are new.
	{
		detection_msgs::DetectedObject o = other.objects.at(i);
		if(o.rotation.x != o.rotation.x || o.rotation.y != o.rotation.y || o.rotation.z != o.rotation.z ||
						o.rotation.w != o.rotation.w)
		{
			ROS_INFO("Filtering out Nan point!");
			continue;
		}

		bool found = false;
		for(size_t j = 0; (j < objects.size()) && !found; j++)
		{
			found = objects.at(j).checkForMatchAndInsert(o,maxDistance,blockRotationModulo90Degree);
			if(found)
			{
				objects.at(j).updated = true;
				ROS_INFO("Found an object");
			}

		}
		if(!found)
		{
			objects.push_back(MapObject(other.objects.at(i)));
			ROS_INFO("Adding new Point!");
		}
	}
	objectsToPublish.objects.clear();

	std::cout << "Points: [";
	for(int k = 0; k < (int)objects.size(); k++)
	{
		if(!objects.at(k).updated)
		{
			objects.at(k).degregate();
			if(objects.at(k).hasLowProbability())
			{
				objects.at(k) = objects.at(objects.size()-1);
				objects.resize(objects.size()-1);
				std::cout << "dead, ";
				k--;
				continue;
			}
		}

		if(k < (int)objects.size())
		{
			std::cout << objects.at(k).obj.probability << ", ";
			if(objects.at(k).obj.probability < 0.3)
			{
				markerManager->publishMarker(objects.at(k).position,TARGET_FRAME,k,255,0,0,"_reallyBad",2.0);
			}else if(objects.at(k).obj.probability < 0.75)
			{
				markerManager->publishMarker(objects.at(k).position,TARGET_FRAME,k,255,255,0,"_bad",2.0);
			}else
			{
				detection_msgs::DetectedObject o = objects.at(k).get();

				if(o.rotation.x != o.rotation.x || o.rotation.y != o.rotation.y || o.rotation.z != o.rotation.z ||
						o.rotation.w != o.rotation.w)
				{
					continue;
				}
				if(o.graspPoint.x == 0.0 && o.graspPoint.y == 0.0 && o.graspPoint.z == 0.0)
				{
					continue;
				}

				markerManager->publishMarker(objects.at(k).position,TARGET_FRAME,k,0,255,0,"_good",2.0);
				markerManager->publishArrow(tf2::Transform(objects.at(k).rotation,objects.at(k).position),TARGET_FRAME,10+k,0.05,1,1,1,"rotation",2.0);
				publishBoxRepresentingObject(objects.at(k),k);
				objectsToPublish.objects.push_back(o);
			}
		}
	}
	std::cout << "]\n";
}

void objectsCallback(const detection_msgs::DetectedObjects::ConstPtr& obj)
{
	std::cout << "Objects callback " << obj->objects.size() << std::endl;
	aggregateObjects(*obj);
}

int main(int argc, char* argv[]) {

	if(argc > 1)
	{
		childNodeName = argv[1];
		if(argc > 2)
		{
			blockRotationModulo90Degree = (std::string(argv[2]) == "true" || std::string(argv[2]) == "True" || std::string(argv[2]) == "1");

			if(argc > 3)
			{
				QUEUE_SIZE_LENGTH = (size_t)std::atoi(argv[3]);
			}
		}
	}else{
		std::cout << "Usage: rosrun detection graspPointBuffer [childNodeName] [maxBufferSize]\n";
	}
	std::cout << "Child Node= " << childNodeName << " Rotation Modulo 90 Degree= " << blockRotationModulo90Degree << " Buffer Size= " << QUEUE_SIZE_LENGTH << std::endl;

	ros::init(argc, argv, "~");
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	markerManager = std::make_unique<MarkerHelper>(nh,"/" + childNodeName + "/markerSave","objects");
	ros::Duration(1.0).sleep();
	ros::Subscriber sub = nh->subscribe("/" + childNodeName + "/objects", 20, objectsCallback);
	ros::Publisher objectsPub = nh->advertise<detection_msgs::DetectedObjects>("/" + childNodeName + "/objectsSave",2);

	ros::Rate r(4);
	ROS_INFO("Starting loop!");
	while (ros::ok())
	{
		if(!init) objectsPub.publish(objectsToPublish);
		ros::spinOnce();
		r.sleep();
	}
}
