/*
 * MarkerManager.cpp
 *
 *  Created on: May 20, 2019
 *      Author: philipf
 */

#include "../include/marker_helper/markerHelper.h"


MarkerHelper::MarkerHelper(std::shared_ptr<ros::NodeHandle> node, std::string topic, std::string nameSpace):nh(node),ns(nameSpace)
{
	markerPub = nh->advertise<visualization_msgs::Marker>(topic,200);
}
void MarkerHelper::publishMarker(geometry_msgs::Point pos, std::string frame, int id, double r, double g, double b, const std::string suffix, double lifetime)
{
	publishMarker(tf2::Vector3(pos.x,pos.y,pos.z),frame,id,r,g,b,suffix,lifetime);
}
void MarkerHelper::publishMarker(tf2::Vector3 pos, std::string frame, int id, double r, double g, double b, const std::string suffix, double lifetime)
{
	if(markerPub.getNumSubscribers() == 0) return;
	if(frame == "") return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns + suffix;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::SPHERE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = pos.x();
	marker.pose.position.y = pos.y();
	marker.pose.position.z = pos.z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.002;
	marker.scale.y = 0.002;
	marker.scale.z = 0.0005;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration(lifetime);
	markerPub.publish(marker);
}
void MarkerHelper::publishArrow(const tf2::Transform t, const std::string frame,
				const int id, const double scale, const double r, const double g, const double b, const std::string suffix, const double lifetime)
{
	if(markerPub.getNumSubscribers() == 0) return;
	if(frame == "") return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns + suffix;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = t.getOrigin().x();
	marker.pose.position.y = t.getOrigin().y();
	marker.pose.position.z = t.getOrigin().z();
	marker.pose.orientation.x = t.getRotation().x();
	marker.pose.orientation.y = t.getRotation().y();
	marker.pose.orientation.z = t.getRotation().z();
	marker.pose.orientation.w = t.getRotation().w();

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = scale;
	marker.scale.y = scale * 0.1;
	marker.scale.z = scale * 0.1;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 0.7;

	marker.lifetime = ros::Duration(lifetime);
	markerPub.publish(marker);
}
void MarkerHelper::publishBox(const tf2::Transform pose, const std::string frame, const tf2::Vector3 scale,
							   const int id, const double r, const double g, const double b, const std::string suffix, const double lifetime)
{
	if(markerPub.getNumSubscribers() == 0) return;
	if(frame == "") return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns + suffix;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::CUBE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = pose.getOrigin().x();
	marker.pose.position.y = pose.getOrigin().y();
	marker.pose.position.z = pose.getOrigin().z();
	marker.pose.orientation.x = pose.getRotation().x();
	marker.pose.orientation.y = pose.getRotation().y();
	marker.pose.orientation.z = pose.getRotation().z();
	marker.pose.orientation.w = pose.getRotation().w();

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = scale.x();
	marker.scale.y = scale.y();
	marker.scale.z = scale.z();

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 0.7;

	marker.lifetime = ros::Duration(lifetime);
	markerPub.publish(marker);
}
void MarkerHelper::publishBox(const tf2::Vector3 start, const tf2::Vector3 end, const std::string frame,
								const int id, const double r, const double g, const double b, const double alpha, const std::string suffix, const double lifetime)
{
	if(markerPub.getNumSubscribers() == 0) return;
	if(frame == "") return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns + suffix;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::CUBE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	tf2::Vector3 diameter = (end - start);
	tf2::Vector3 center = start + (diameter/2.0);

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = center.x();
	marker.pose.position.y = center.y();
	marker.pose.position.z = center.z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = std::fabs(diameter.x());
	marker.scale.y = std::fabs(diameter.y());
	marker.scale.z = std::fabs(diameter.z());

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = alpha;

	marker.lifetime = ros::Duration(lifetime);
	markerPub.publish(marker);
}

void MarkerHelper::publishLines(const std::vector<tf2::Vector3> points, const std::vector<tf2::Vector3> normal, const double length, std::string frame,
				const int id, const double r, const double g, const double b, const std::string suffix, const double lifetime)
{
	if(markerPub.getNumSubscribers() == 0) return;
	if(frame == "") return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns + suffix;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::LINE_LIST;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	//marker.scale.x = std::fabs(diameter.x());
	//marker.scale.y = std::fabs(diameter.y());
	//marker.scale.z = std::fabs(diameter.z());

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 1.0;

	for(size_t i = 0; i < points.size(); i++)
	{
		geometry_msgs::Point p;
		p.x = points.at(i).x(); p.y = points.at(i).y(); p.z = points.at(i).z();
		marker.points.push_back(p);
		p.x = points.at(i).x() + (normal.at(i).x() * length);
		p.y = points.at(i).y() + (normal.at(i).y() * length);
		p.z = points.at(i).z() + (normal.at(i).z() * length);
		marker.points.push_back(p);
	}

	marker.lifetime = ros::Duration(lifetime);
	markerPub.publish(marker);
}

void MarkerHelper::publishText(const tf2::Vector3 pos, const double size, const std::string text, const std::string frame,
						const int id, const double r, const double g, const double b, const std::string suffix, const double lifetime)
{
	if(markerPub.getNumSubscribers() == 0) return;
	if(frame == "") return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns + suffix;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = pos.x();
	marker.pose.position.y = pos.y();
	marker.pose.position.z = pos.z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.z = size;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration(lifetime);
	markerPub.publish(marker);
}
void MarkerHelper::clearAll(const std::string suffix)
{
	if(markerPub.getNumSubscribers() == 0) return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns + suffix;
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::DELETEALL;

	markerPub.publish(marker);
}
