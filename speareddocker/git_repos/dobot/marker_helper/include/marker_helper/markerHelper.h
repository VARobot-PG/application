/*
 * marker_helper.h
 *
 *  Created on: May 20, 2019
 *      Author: philipf
 */

#ifndef SRC_DOBOT_DOBOT_PROGRAMS_INCLUDE_LIBRARIES_MARKERHELPER_H_
#define SRC_DOBOT_DOBOT_PROGRAMS_INCLUDE_LIBRARIES_MARKERHELPER_H_
#include <tf2/LinearMath/Transform.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

	class MarkerHelper{
	public:
		MarkerHelper(const std::shared_ptr<ros::NodeHandle> node, const std::string topic, const std::string nameSpace);
		void publishMarker(const geometry_msgs::Point pos, const std::string frame,
				const int id, const double r, const double g, const double b, const std::string suffix="", const double lifetime=5.0);
		void publishMarker(const tf2::Vector3 pos, const std::string frame,
				const int id, const double r, const double g, const double b, const std::string suffix="", const double lifetime=5.0);
		void publishArrow(const tf2::Transform t, const std::string frame,
				const int id, const double scale, const double r, const double g, const double b, const std::string suffix="", const double lifetime=5.0);
		void publishBox(const tf2::Transform pose, const std::string frame, const tf2::Vector3 scale,
				const int id, const double r, const double g, const double b, const std::string suffix="", const double lifetime=5.0);
		void publishBox(const tf2::Vector3 start, const tf2::Vector3 end, const std::string frame,
				const int id, const double r, const double g, const double b, const double alpha=0.7, const std::string suffix="", const double lifetime=5.0);
		void publishLines(const std::vector<tf2::Vector3> points, const std::vector<tf2::Vector3> normal, const double length, std::string frame,
				const int id, const double r, const double g, const double b, const std::string suffix="", const double lifetime=5.0);
		void publishText(const tf2::Vector3 pos, const double size, const std::string text, const std::string frame,
						const int id, const double r, const double g, const double b, const std::string suffix="", const double lifetime=5.0);
		void clearAll(const std::string suffix="");
	private:
		const std::shared_ptr<ros::NodeHandle> nh;
		const std::string ns;
		ros::Publisher markerPub;
	};



#endif /* SRC_DOBOT_DOBOT_PROGRAMS_INCLUDE_LIBRARIES_MARKERHELPER_H_ */
